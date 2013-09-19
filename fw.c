#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>
#include "iomacros.h"
#include "uart.h"
#include "onewire.h"
#include "ds18x20.h"

#define RELAY_PIN 4,C
#define BOOT_RELAY_PIN 5,C

#define BAUD 9600

// 0 - on by default (when pin is off), 1 - off
#define NORMALLY_OPEN_RELAY 1

// on my relay block on/off logic is inversed (?), so on<-->off functions considering relays

uint16_t EEMEM low_boundary = 220;
uint16_t EEMEM high_boundary = 225;
int8_t EEMEM cur_state_saved = '0';

volatile int16_t low;
volatile int16_t high;

volatile int16_t cur_temp = -1000;
volatile char cur_state = '0';

volatile int reboot = 0;

volatile int timer_count = 0;

volatile int unsynced_count = 0;

void measure_temp(void) {
  DS18X20_start_meas(DS18X20_POWER_EXTERN, NULL);
  _delay_ms(DS18B20_TCONV_12BIT);
  int16_t t;
  if(DS18X20_read_decicelsius_single(DS18B20_FAMILY_CODE, &t) == DS18X20_OK)
    cur_temp = t;
}

int16_t get_temp(void) {
  if(cur_temp == -1000) {
    measure_temp();
  }
  return cur_temp;
}

void check(void) {
  measure_temp();
  if(cur_temp < low) { //turn on
#if NORMALLY_OPEN_RELAY
    off(RELAY_PIN);
#else
    on(RELAY_PIN);
#endif
    cur_state = '1';
  }
  if(cur_temp > high) {
#if NORMALLY_OPEN_RELAY
    on(RELAY_PIN);
#else
    off(RELAY_PIN);
#endif
    cur_state = '0';
  }

  eeprom_update_byte(&cur_state_saved, cur_state);

  unsynced_count++;
  if(unsynced_count > 10) {
    // master device didn't connected more than 10 minutes - reboot it
    unsynced_count = 0;
    reboot = 1;
  }
}

ISR(TIMER1_COMPA_vect)
{
  timer_count++;
  if(timer_count == 12) {
    timer_count = 0;

    check();

    if(reboot == 1) {
      reboot = 0;
      off(BOOT_RELAY_PIN);
      _delay_ms(5000);
      on(BOOT_RELAY_PIN);
    }
  }
}

void outs(const char *str) {
  uart0_puts(str);
}

char inchar(void) {
  uint16_t res = uart0_getc();
  while((res >> 8) != 0) {
    res = uart0_getc();
  }
  return (char)res;
}

char inchar_nowait(void) {
  uint16_t res = uart0_getc();
  return (char)res;
}

int main(void) {
  out(RELAY_PIN);
  out(BOOT_RELAY_PIN);

  low = (int16_t)eeprom_read_word(&low_boundary);
  high = (int16_t)eeprom_read_word(&high_boundary);
  cur_state = (char)eeprom_read_byte(&cur_state_saved);
  if(cur_state == '0')
    on(RELAY_PIN);
  else
    off(RELAY_PIN);
  on(BOOT_RELAY_PIN);

  // init timer
  OCR1A = 39062; // 1 time in 5 s
  TCCR1B |= (1 << WGM12);
  TIMSK |= (1 << OCIE1A);
  TCCR1B |= (1 << CS12) | (1 << CS10); // prescaler 1024

  uart0_init(UART_BAUD_SELECT(BAUD, F_CPU));

  sei();

  ow_reset();

  check();

  // enable watchdog timer at longest period (about 2.1s)
  wdt_enable(WDTO_2S);

  char buf[16];
  char in_char;
  char *s;
  while(1) {
    wdt_reset();
    in_char = inchar_nowait();
    switch(in_char) {
    case 't': // return current temperature
      sprintf(buf, "%d\n", get_temp());
      outs(buf);
      unsynced_count = 0;
      break;
    case 'b': // wait about 1 min and blink BOOT_RELAY_PIN
      reboot = 1;
      timer_count = 0;
      break;
    case 'c': // return current state (0, 1, .)
      buf[0] = cur_state;
      buf[1] = 0;
      outs(buf);
      break;
    case 's': // set boundaries, 'sh105' or 'sl01' (for high-10.5, low-0.1 C)
      s = buf;
      while(1) {
        in_char = inchar();
        *s++ = in_char;
        if(in_char == '\n' || in_char == '\r' || in_char == ' ')
          break;
      }
      *s = 0;
      uint16_t temp;
      char w;
      sscanf(buf, "%c%d", &w, &temp);
      if(w == 'h') {
        high = temp;
        eeprom_update_word(&high_boundary, temp);
      }
      if(w == 'l') {
        low = temp;
        eeprom_update_word(&low_boundary, temp);
      }
      break;
    case 'g': // get boundaries, 'gh' or 'gl'
      in_char = inchar();
      if(in_char == 'h') {
        sprintf(buf, "%d\n", high);
        outs(buf);
      }
      if(in_char == 'l') {
        sprintf(buf, "%d\n", low);
        outs(buf);
      }
      break;
    }
  }
}

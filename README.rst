Description
-----------

Firmware for thermostat based on Atmel microcontroller with a DS18B20 temperature sensor and a relay.

Tested with ATmega8L.

Web-interface to control settings - http://bitbucket.org/ForeverYoung/thermostat-web

MCU is connected by a standard scheme, without external oscillator.

Relay is connected to PC4 pin.

DS18B20 is powered with external supply (not parasite-powered).

Control by UART pins PD0, PD1, 9600 8N1.

Protocol:
 't':
   Return current temperature. Temperature in deciCelsius.
 'c':
   Return current state ('1' - on, '0' - off (logical, not dependent on parameter NORMALLY_OPEN_RELAY), '.' - not changed).
 's':
   Set boundaries - 'sh' - set high boundary, 'sl' - set low boundary. Eg. 'sl215 ', 'sh220 ' (space or newline is required after number).
 'g':
   Get boundaries, 'gh', 'gl'.
 'b':
   Hard-reset of controlling device (not tested!). Pin PC5.

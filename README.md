# PlantTimer

Functionalities:
Use real-time clock (DS3231) to manage lighting, air and temperature of a plant growing environment with 8x relay board.

Programming is done via serial commands:
T<unixtimestamp>; sets current time
B<unixtimestamp>; sets bloom lighting scheme start

Features completed:
- Setting of time and bloom start by serial command
- Growth light time scheme based output switching, possible to use "gas lantern" method
- Bloom light switching with daily time changes (shorten days/nights by x minutes)
- 730 nm lighting (pre- and after glow duration at bloom light day end)
- Irrigation timer
- Temperature sensor identification
- Temperature controlled exhaust fan throttle with hysteresis

TODO:
- Some kind of display output

Most libraries can be installed from within the
Arduino IDE with the library manager.

External libraries used:
- Time 1.5.0: https://github.com/PaulStoffregen/Time
- TimeAlarms 1.4.0: https://github.com/PaulStoffregen/TimeAlarms
- OneWire 2.3.2: https://github.com/PaulStoffregen/OneWire
- DS18B20 temperature conversion: https://github.com/milesburton/Arduino-Temperature-Control-Library

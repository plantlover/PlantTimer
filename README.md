# PlantTimer

Functionalities:
Use real-time clock (DS3231) to manage lighting,
air and temperature of a plant growing environment
with 8x relay board connected via I2C gpio expander
and status display on 20x4 I2C LCD.

Programming is done via serial commands:
T<unixtimestamp>; sets time
B<unixtimestamp>; sets bloom lighting scheme start

Features completed:
- Setting of time and bloom start by serial command
- Growth light time scheme based output switching
- Bloom light switching with daily time changes

TODO:
- 730 nm lighting
- Temperature sensor identification
- Temperature controlled exhaust fan throttle
- Timed air circulation
- MAX_BLOOM_DAYS is funky
- nicen LCD output

Most libraries can be installed from within the
Arduino IDE with the library manager.

External libraries used:
- Time 1.5.0: https://github.com/PaulStoffregen/Time
- TimeAlarms 1.4.0: https://github.com/PaulStoffregen/TimeAlarms
- OneWire 2.3.2: https://github.com/PaulStoffregen/OneWire
- DS18B20 temperature conversion: https://github.com/milesburton/Arduino-Temperature-Control-Library
- LiquidCrystal_I2C 1.1.2: https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
- I2C GPIO Expander: https://github.com/sumotoy/gpio_expander

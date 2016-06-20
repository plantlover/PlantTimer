/* PlantTimer

    Functionalities:
    Use real-time clock (DS3231) to manage lighting,
    air and temperature of a plant growing environment
    with 8x relay board.

    Programming is done via serial commands:
    T<unixtimestamp>; sets time
    B<unixtimestamp>; sets bloom lighting scheme start

    Features completed:
    - Setting of time and bloom start by serial command
    - Growth light time scheme based output switching
    - Bloom light switching with daily time changes
    - 730 nm lighting for X minutes before and after
      bloom light switch off
    - Temperature sensor identification
    - Exhaust fan throttle if temperature is low enough

    TODO:
    - Timed air circulation
    - MAX_BLOOM_DAYS is funky
    - Another Arduino for LCD display

    All libraries can be installed from within the
    Arduino IDE with the library manager.

    Last compiled with Arduino IDE 1.6.8

    External libraries:
    - Time 1.5.0: https://github.com/PaulStoffregen/Time
    - TimeAlarms 1.4.0: https://github.com/PaulStoffregen/TimeAlarms
    - OneWire 2.3.2: https://github.com/PaulStoffregen/OneWire
    - DallasTemperature 3.7.6: https://github.com/milesburton/Arduino-Temperature-Control-Library
*/

#include <Time.h>
#include <TimeAlarms.h>
#include <DS1307RTC.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <EEPROM.h>

/* Definition of Arduino digital pins */
/* Not mentioned are A4, A5, because they're standard I2C used by Wire.h */

#define PIN_ONEWIRE 10            // OneWire data pin, add 4k7 pullup resistor between DATA and +5V
#define PIN_AIR_OUTLET_THROTTLE 9 // Relay IN8: Exhaust air fan - output pin HIGH means throttle down exhaust air fan
#define PIN_PUMP_IRRIGATION 8     // Relay IN7: Water pump (for hydro-system)
#define PIN_PUMP_MEASUREMENT 7    // Relay IN6: Circulation pump for taking pH- and EC-measurements
#define PIN_LIGHT_1 6             // Relay IN5: Grow
#define PIN_LIGHT_2 5             // Relay IN4: Bloom
#define PIN_LIGHT_3 4             // Relay IN3: 730 nm lights
#define PIN_LIGHT_COOLER 3        // Relay IN2: Cooling fan for LED
#define PIN_AIR_CIRCULATION 2     // Relay IN1: Circulation fans inside grow compartments

/* Serial command headers */
#define HEADER_TIMESET "T"
#define HEADER_BLOOMSTARTSET "B"

/* 8-relay-board needs HIGH to OPEN the relay, so for readability, we redefine the use of HIGH/LOW */
#define RELAYON LOW
#define RELAYOFF HIGH

/* Initialize OneWire and temperature sensors */
OneWire ow(PIN_ONEWIRE);
DallasTemperature tempSensors(&ow);

/* DS18B20 OneWire IDs
   Don't worry if you don't know them yet, they will show up on serial port.
*/
uint8_t tempSensorAirCirculation[8] = { 0x28, 0x16, 0xA3, 0xAA, 0x04, 0x00, 0x00, 0xA6 };
uint8_t tempSensorLight[8]          = { 0x28, 0xD2, 0x89, 0x2A, 0x04, 0x00, 0x00, 0x79 };
uint8_t tempSensorElectronics[8]    = { 0x28, 0xB8, 0x75, 0xAA, 0x04, 0x00, 0x00, 0x2B };

/* Light schemes in minutes */
// Light1: Growth light (12+1 method)
// => 12 hours ON, 5.5 hours OFF, 1 hour ON, 5.5 hours OFF, repeat
// The array HAS to sum up to 24 hours, but can be of any even count / growth light periods.
// Only two growth light periods (typically ON for 1080 minutes, OFF for 360 minutes) are
// also possible.
#define GROWTH_LIGHT_PERIODS 4
int growthLightScheme[GROWTH_LIGHT_PERIODS] = { 720, 330, 60, 330 };
byte currentGrowthLightPeriod;

// Time of day growth light scheme should start (example: 4:20 pm)
byte growthLightStartTimeHour = 16;
byte growthLightStartTimeMinute = 20;

// Light2: Bloom light
// => 12 hours ON, 12 hours OFF, repeat
// These numbers can be set freely, so for example a 23 hour day is possible.
// Only ONE ON and ONE OFF period are possible (and make sense).
int bloomLightScheme[2] = { 780, 690 };

// Daily reduce duration of daytime by X minutes after bloom day Y.
// Set to 0 if unwanted.
byte reductionOfDaytimeInMinutes = 3;
// Daily prolong duration of nighttime by X minutes every day
byte prolongNighttimeInMinutes = 0;
// Start with day-length modifications at bloom day X
byte startOfDaytimeReduction = 10;

// If bloom mode is not active, switch bloom light output according to growth light scheme
bool switchBloomLightInGrowthSchemeIfNotInBloom = true;

// Resets bloom mode to off if your Arduino sits in the shelf for a while
#define MAX_BLOOM_DAYS 120

// Light3: 730 nm LEDs
// First number is the on-time BEFORE bloom light turns off,
// second number is the time it stays on after
short sleepLightScheme[2] = { 10, 15 };

/* Irrigation scheme in seconds */
// => 0.5 minutes on, 10 minutes off, repeat
unsigned long irrigationScheme[2] = { 40, 500 };

// Exhaust fan temperature hysteresis
// Turns exhaust fan to low if plant room temperature is below 23°C
// Turns back to high at hysteresis value (+2 K = 25°C)
int minRoomTemp = 23;
int exhaustFanHysteresis = 2;

/* FROM HERE ON COMES SYSTEM STUFF */
String serialInputString = "";         // a string to hold incoming serial data
boolean serialStringComplete = false;  // whether the string is complete

byte bloomDayCounter = 0;  // is filled by calculating forwards from storedSettings.bloomStart
bool bloomLightStatus = 0; // same as above
long secondsToNextGrowthLightSwitch = 0; // Calculated on startup to set first planned light switch
long secondsToNextBloomLightSwitch = 0;  // same as above
long secondsToNextSleepLightSwitch = 0;  // same as above

bool fanThrottleActive = 0;

/* Data object for settings that will be stored in EEPROM */
struct storedDataObject {
  // Remembers when bloom period was started or WILL BE started
  // Has to be set via serial command "T<unixtimestamp>;"
  time_t bloomStart;  // time value with the complete date of desired bloom mode start
};

// Create instance of EEPROM data-model
storedDataObject storedSettings;

// Startup routine
void setup() {
  // Set up output pins for relay switching
  for (int pin=2; pin < 9; pin++) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
  
  // Open serial connection
  Serial.begin(57600);
  // Reserve 50 bytes for the serialInputString
  serialInputString.reserve(50);

  // Set up real-time clock connection to "Time" library
  setSyncProvider(RTC.get);
  setSyncInterval(1);

  // Write current time into variable, because now() seems a lot of work behind the curtains
  time_t timeNow = now();

  // Initialize temperature sensors
  tempSensors.begin();

  // Only continue if RTC is working correctly
  if (timeStatus() == timeSet) {
    Serial.print(F("Time of startup: "));
    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());

    Serial.print(F(" or "));
    Serial.println(timeNow);


    // Fill storedSettings with data from EEPROM storage
    EEPROM.get(0, storedSettings);

    Serial.print(F("Bloom start: "));
    Serial.println(storedSettings.bloomStart);

    // Find OneWire temperature sensors
    printOneWireDevices();

    // Start the first calculation which light period we should be at now
    getGrowthLightPeriod();

    /* TIMER SETUP
        ========
    */

    // if growth light period is even, means growth light should be on
    // => turn on growth light
    if ((currentGrowthLightPeriod & 1) == 0) {
      turnOnGrowthLight();
    }
    // else if growth light period is odd
    // => set up next growth light ON event
    else {
      Alarm.timerOnce(secondsToNextGrowthLightSwitch, turnOnGrowthLight);
      Serial.print(F("Growth light ON in seconds: "));
      Serial.println(secondsToNextGrowthLightSwitch);
    }

    // Bloom light setup
    // Only set up bloom light scheme if bloomStart is set
    // and not in the future
    if ((storedSettings.bloomStart > 0) && ((timeNow - storedSettings.bloomStart) > 0)) {
      getBloomLightStatus();

      if (!bloomLightStatus) { // if current period is 0, turn on bloom light
        turnOnBloomLight();
        Serial.print(F("Bloom light OFF in seconds: "));
        Serial.println(secondsToNextBloomLightSwitch);
        Serial.print(F("Sleep light ON in seconds: "));
        Serial.println(secondsToNextBloomLightSwitch - (sleepLightScheme[0] * SECS_PER_MIN));
      }
      else { // Light scheme period 1 means light is currently OFF
        Alarm.timerOnce(secondsToNextBloomLightSwitch, turnOnBloomLight);
        Serial.print(F("Set up bloom light ON in seconds: "));
        Serial.println(secondsToNextBloomLightSwitch);
      }
      // Check if sleep light should be on
      if (getSleepLightStatus() == true) {
        digitalWrite(PIN_LIGHT_3, RELAYON);
        Alarm.timerOnce(secondsToNextSleepLightSwitch, turnOffSleepLight);
      }
      else {
        Alarm.timerOnce(secondsToNextSleepLightSwitch, turnOnSleepLight);
      }
      Serial.print(F("Bloom day counter: "));
      Serial.println(bloomDayCounter);
    }
    
    // Request reading of temperatures on OneWire bus every 5 seconds
    Alarm.timerRepeat(5, requestOneWireTemperatures);
    
    // Print all OneWire temperature sensors every 180 seconds
    Alarm.timerRepeat(180, printOneWireDevices);

    // Turn on irrigation
    turnOnIrrigation();
  }
  else {
    Serial.println(F("Fix the RTC!"));
  }
}



void loop() {
  Alarm.delay(1);

  // If a serial command has been received completely
  if (serialStringComplete) {
    serialInputString.replace(";", ""); // remove trailing semicolon

    // Set time if serial string starts with defined HEADER_TIMESET (default 'T')
    if (serialInputString.startsWith(HEADER_TIMESET) == true) {
      serialInputString.replace(HEADER_TIMESET, "");
      setSerialTime(serialInputString.toInt());
    }

    // Set bloom start timestamp if serial string starts with defined HEADER_BLOOMSTARTSET (default 'B')
    else if (serialInputString.startsWith(HEADER_BLOOMSTARTSET) == true) {
      serialInputString.replace(HEADER_BLOOMSTARTSET, "");
      setBloomStart(serialInputString.toInt());
    }

    else {
      Serial.println(F("I don't understand what you're saying"));
    }

    // clear the string:
    serialInputString = "";
    serialStringComplete = false;
  }

  // Exhaust fan throttle with hysteresis
  if (tempSensors.getTempC(tempSensorAirCirculation) <= minRoomTemp && !fanThrottleActive) {
    // Switch on throttle for exhaust fan -> reduce air volume
    digitalWrite(PIN_AIR_OUTLET_THROTTLE, RELAYON);
    fanThrottleActive = true;
    Serial.println(F("Turned down fan"));
  }
  else if ((tempSensors.getTempC(tempSensorAirCirculation) > (minRoomTemp + exhaustFanHysteresis)) && fanThrottleActive == true) {
    digitalWrite(PIN_AIR_OUTLET_THROTTLE, RELAYOFF);
    fanThrottleActive = false;
    Serial.println(F("Turned up fan"));
  }

}

// Counts forward from one day before the daily lighting scheme start
// and looks for the current lighting period we're at
void getGrowthLightPeriod() {
  // Create time element for Alarm setup of growth light
  time_t timeNow = now();
  tmElements_t dailyGrowthLightStartTime;
  breakTime(timeNow, dailyGrowthLightStartTime);
  // Modify time element with variable start time (default: 16:20)
  dailyGrowthLightStartTime.Hour = growthLightStartTimeHour;
  dailyGrowthLightStartTime.Minute = growthLightStartTimeMinute;
  dailyGrowthLightStartTime.Second = 0;

  // Subtract one day from growth light start time
  // Set up variable to count through growth light periods
  unsigned long sumOfGrowthLightPeriods = makeTime(dailyGrowthLightStartTime) - SECS_PER_DAY;
  byte count = 0;

  // Comb through on/off periods since one day earlier until we know which lighting period we're at now
  while (sumOfGrowthLightPeriods <= timeNow) {
    sumOfGrowthLightPeriods += growthLightScheme[count] * SECS_PER_MIN;
    if (sumOfGrowthLightPeriods <= timeNow) {  // If already past now(), don't count next light period
      currentGrowthLightPeriod = count;
      if (count < GROWTH_LIGHT_PERIODS-1) count++;
      else count = 0;
    }
  }
  secondsToNextGrowthLightSwitch = sumOfGrowthLightPeriods - timeNow;
  currentGrowthLightPeriod = count;
}

// Counts forward from timestamp storedSettings.bloomStart
// Sums up all nights and days including optional day-/nighttime duration modification
// Calculates effective bloom days (since there might have been shorter-than-24-hour days)
void getBloomLightStatus() {
  time_t timeNow = now();
  unsigned long sumOfBloomLightPeriods = storedSettings.bloomStart;

  while (sumOfBloomLightPeriods <= timeNow) {
    sumOfBloomLightPeriods += bloomLightScheme[bloomLightStatus] * SECS_PER_MIN;
    if (sumOfBloomLightPeriods <= timeNow) { // If already past now(), don't count next light period
      if (!bloomLightStatus) {
        bloomLightStatus = true;
        if (bloomDayCounter >= startOfDaytimeReduction) {
          sumOfBloomLightPeriods -= reductionOfDaytimeInMinutes * SECS_PER_MIN;
        }
      }
      else {
        bloomLightStatus = false;
        if (bloomDayCounter >= startOfDaytimeReduction) {
          sumOfBloomLightPeriods += prolongNighttimeInMinutes * SECS_PER_MIN;
        }
        bloomDayCounter++;
      }
    }
  }

  secondsToNextBloomLightSwitch = sumOfBloomLightPeriods - timeNow;
}

boolean getSleepLightStatus() {
  time_t secondsSinceLastBloomLightSwitch;

  secondsSinceLastBloomLightSwitch = abs(secondsToNextBloomLightSwitch - (bloomLightScheme[bloomLightStatus] * SECS_PER_MIN));

  // If shortly within bloom light OFF, turn on sleep light
  if ((sleepLightScheme[0] * SECS_PER_MIN) < secondsToNextBloomLightSwitch) {
    // set duration until sleep light will be turned OFF
    secondsToNextSleepLightSwitch = secondsToNextBloomLightSwitch + (sleepLightScheme[1] * SECS_PER_MIN);
    return true;
  }
  // If shortly after bloom light OFF, turn on sleep light too
  else if ((sleepLightScheme[1] * SECS_PER_MIN) < secondsSinceLastBloomLightSwitch) {
    // set duration until sleep light will be turned OFF
    secondsToNextSleepLightSwitch = secondsSinceLastBloomLightSwitch + (sleepLightScheme[1] * SECS_PER_MIN);
    return true;
  }
  // If in between bloom light periods, don't turn on sleep light right now, but set duration until next turn ON
  else {
    // set duration until sleep light will be turned ON
    secondsToNextSleepLightSwitch = secondsToNextBloomLightSwitch - (sleepLightScheme[0] * SECS_PER_MIN);
    return false;
  }
}

void turnOnIrrigation() {
  digitalWrite(PIN_PUMP_IRRIGATION, RELAYON);
  Alarm.timerOnce(irrigationScheme[0], turnOffIrrigation);
  Serial.print(F("Turned on irrigation at "));
  Serial.println(now());
}

void turnOffIrrigation() {
  digitalWrite(PIN_PUMP_IRRIGATION, RELAYOFF);
  Alarm.timerOnce(irrigationScheme[1], turnOnIrrigation);
  Serial.print(F("Turned off irrigation at "));
  Serial.println(now());
}

void turnOnGrowthLight() {
  digitalWrite(PIN_LIGHT_1, RELAYON);
  Serial.print(F("Turned on growth light"));
  if (switchBloomLightInGrowthSchemeIfNotInBloom) {
    if ((storedSettings.bloomStart == 0) || (now() < storedSettings.bloomStart)) {
      digitalWrite(PIN_LIGHT_2, LOW);
      Serial.print(F(" and bloom light"));
    }
  }
  Serial.print(" at ");
  Serial.println(now());

  getGrowthLightPeriod();
  Alarm.timerOnce(secondsToNextGrowthLightSwitch, turnOffGrowthLight);
  Serial.print(F("Set up growth light OFF in seconds: "));
  Serial.println(secondsToNextGrowthLightSwitch);
}

void turnOffGrowthLight() {
  digitalWrite(PIN_LIGHT_1, RELAYOFF);
  Serial.print(F("Turned off growth light"));
  if (switchBloomLightInGrowthSchemeIfNotInBloom) {
    if ((storedSettings.bloomStart == 0) || (now() < storedSettings.bloomStart)) {
      digitalWrite(PIN_LIGHT_2, RELAYOFF);
      Serial.print(F(" and bloom light"));
    }
  }
  Serial.print(F(" at "));
  Serial.println(now());

  getGrowthLightPeriod();
  Alarm.timerOnce(secondsToNextGrowthLightSwitch, turnOnGrowthLight);
  Serial.print(F("Set up growth light ON in seconds: "));
  Serial.println(secondsToNextGrowthLightSwitch);
}

void turnOnBloomLight() {
  digitalWrite(PIN_LIGHT_2, RELAYON);
  Serial.println(F("Turned on bloom light"));
  // calculate and set the next OFF-period
  getBloomLightStatus();
  Alarm.timerOnce(secondsToNextBloomLightSwitch, turnOffBloomLight);
}

void turnOffBloomLight() {
  digitalWrite(PIN_LIGHT_2, RELAYOFF);
  Serial.println(F("Turned off bloom light"));
  // calculate and set the next ON-period
  getBloomLightStatus();
  Alarm.timerOnce(secondsToNextBloomLightSwitch, turnOnBloomLight);
}

void turnOnSleepLight() {
  Serial.println(F("Turned on sleep light"));
  digitalWrite(PIN_LIGHT_3, RELAYON);
  getSleepLightStatus();
  Alarm.timerOnce(secondsToNextSleepLightSwitch, turnOffSleepLight);
}

void turnOffSleepLight() {
  Serial.println(F("Turned off sleep light"));
  digitalWrite(PIN_LIGHT_3, RELAYOFF);
  getSleepLightStatus();
  Alarm.timerOnce(secondsToNextSleepLightSwitch, turnOnSleepLight);
}

void requestOneWireTemperatures() {
  tempSensors.requestTemperatures();
}

void printOneWireDevices() {
  uint8_t address[8];
  byte count = tempSensors.getDeviceCount();

  // Read temperatures from devices
  requestOneWireTemperatures();

  Serial.print(F("Sensors found: "));
  Serial.println(count);

  for (byte i = 0; i < count; i++) {
    Serial.print(F("Sensor "));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(tempSensors.getTempCByIndex(i));
    Serial.print(F(" at address { "));
    tempSensors.getAddress(address, i);
    for (uint8_t i = 0; i < 8; i++) {
      Serial.print(F("0x"));
      if (address[i] < 0x10) Serial.print("0");
      Serial.print(address[i], HEX);
      if (i != 7) Serial.print(F(", "));
    }
    Serial.println(F(" };"));
  }
}


// Called by serial command "T<unix timestamp;"
// Sets the time of the RTC clock
void setSerialTime(unsigned long timeInput) {
#define MIN_TIME 1451606400UL // Jan 1 2016

  if (timeInput >= MIN_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
    Serial.print(F("Time before set to: "));
    Serial.println(now());
    setTime(timeInput); // Sync Arduino clock to the time received on the serial port
    RTC.set(timeInput); // set the RTC and the system time to the received value
    Serial.print(F("Time now set to: "));
    Serial.println(now());
  }
  else if (!timeInput) {
    Serial.print(F("Current timestamp: "));
    Serial.println(now());
  }
  else {
    Serial.print(F("Not a valid unix timestamp. Use \"date +T%s\" in your shell to get a useful timeset command."));
  }
}

// Called by serial command "B<unix timestamp>;"
// Sets the start date and time of bloom lighting mode
void setBloomStart(time_t serialTimeInput) {
  unsigned long timeInput = serialTimeInput;
  long elapsedBloomSeconds = now() - timeInput;
  int elapsedBloomDays = elapsedDays(elapsedBloomSeconds);
#define MIN_TIME 1451606400UL // Jan 1 2016

  if (timeInput >= MIN_TIME) { // check the integer is a valid time (greater than Jan 1 2016)
    if (elapsedBloomDays < MAX_BLOOM_DAYS) {   // check the set date is not more than MAX_BLOOM_DAYS in the past
      Serial.print(F("Bloom start before set: "));
      Serial.println(storedSettings.bloomStart);
      storedSettings.bloomStart = timeInput;
    }
    else {  // if the desired bloom start day is more than MAX_BLOOM_DAYS in the past, set to 0 and thus disable bloom light scheme
      Serial.println(F("Bloom days more than MAX_BLOOM_DAYS, disabling bloom start"));
      storedSettings.bloomStart = 0;
    }
    EEPROM.put(0, storedSettings);
    EEPROM.get(0, storedSettings);
    Serial.print(F("Bloom start now set to: "));
    Serial.println(storedSettings.bloomStart);
  }
  else if (timeInput == 0) {
    storedSettings.bloomStart = 0;
    EEPROM.put(0, storedSettings);
    EEPROM.get(0, storedSettings);
    Serial.print(F("Bloom start now set to: "));
    Serial.println(storedSettings.bloomStart);
  }
  else if (!timeInput) {
    Serial.println(F("Not a valid unix timestamp. Use \"date +T%s\" in your shell to get a useful timeset command."));
    Serial.print(F("Bloom start currently set to: "));
    Serial.println(storedSettings.bloomStart);
  }
}

// Collects data that comes in via serial connection
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    serialInputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == ';') {
      serialStringComplete = true;
    }
  }
}

#include <TimeLib.h>
#include <TinyGPS.h>
#include <EEPROM.h>

//Object to parse GPS NMEA0183 data
TinyGPS gps;
//Object to access UART0 connected to the GPS
HardwareSerial Uart = HardwareSerial();
//LED pin output
int ledPin = 13;

//latitude/longitude in degrees
long lat, lon;

void setup()  {
  pinMode(ledPin, OUTPUT);
  Uart.begin(9600);
  Serial.begin(9600);
  setSyncProvider(getTeensy3Time);
  delay(5000);
  lat = l_EEPROM2Lat();
  lon = l_EEPROM2Lon();
  Serial.println("Current RTC date:");
  digitalClockDisplay();
  Serial.println("Current EEPROM position:");
  Serial.println("Lat: " + String(lat) + " Lon: " + String(lon));
  bool GPSSynced = false;
  int timeOut = 120;
  
  Serial.println("Starting up GPS and waiting for fix...");
  while (GPSSynced == false && timeOut > 0) {
    blinkLed(1, ledPin);
    timeOut -= 5; //to be matched with the 5000 timeout for reading serial port

    bool newdata = false;
    unsigned long start = millis();

    // Every 5 seconds we print an update
    while (millis() - start < 5000) {
      if (Uart.available()) {
        char c = Uart.read();
        // Serial.print(c);  // uncomment to see raw GPS data
        if (gps.encode(c)) {
          newdata = true;
        }
      }
    }

    if (newdata) {
      unsigned long fix_age;
      int Year;
      byte Month, Day, Hour, Minute, Second;
      // retrieves +/- lat/long in 100000ths of a degree
      gps.get_position(&lat, &lon, &fix_age);
      // time in hhmmsscc, date in ddmmyy
      gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &fix_age);

      if (fix_age == TinyGPS::GPS_INVALID_AGE)
        Serial.println("No fix detected, waiting...");
      else if (fix_age > 5000)
        Serial.println("Warning: possible stale data! waiting...");
      else{
        // set the Time to the latest GPS reading
        Serial.println("Data is current. Setting up the RTC date and time.");
        setTime(Hour, Minute, Second, Day, Month, Year);
        Serial.println("Saving location to EEPROM.");
        l_Lat2EEPROM(lat);
        l_Lon2EEPROM(lon);
        digitalClockDisplay();
        Serial.println("Lat: " + String(lat) + " Lon: " + String(lon));
        GPSSynced = true;
      }
    }
    else
    {
      Serial.println("No GPS data. Waiting " + String(timeOut)); 
    }
  }

  Serial.println("Starting up IMU and waiting accel and gyro calibration...");
  bool IMUDetected = false;
  timeOut = 1;
  while (IMUDetected == false && timeOut > 0) {
    blinkLed(2, ledPin);
    timeOut -= 1;

    delay(1000);
  }

  Serial.println("Compass calibration, move it in every directions...");
  bool magnetoCalibrated = true;
  while (magnetoCalibrated == false) {
    blinkLed(3, ledPin);
    timeOut -= 1;

    delay(1000);
  }
}

void loop() {
  digitalClockDisplay();

  delay(1000);

}


time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

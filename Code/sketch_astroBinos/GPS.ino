#include <TinyGPS.h> //for NMEA0183 GPS data parsing

// Classe utilis�e pour communiquer avec le GPS
class Gps
{
  private:
    //Object to parse GPS NMEA0183 data
    TinyGPS tgps;
    //Object to access UART0 connected to the GPS
    HardwareSerial Uart ;
    //latitude/longitude in degrees
    long latN, lonE;
    unsigned long fix_age, time, date, speed, course;

    uint32_t lastupdate = 0;
    char raw; // raw GPS data
    bool newdata;

  public:
    void Setup();
    bool Read();
    long LatitudeN();
    long LongitudeE();
};

//Sets the UART speed for reading GPS NMEA0183
void Gps::Setup() {
  HardwareSerial Uart = HardwareSerial();
  Uart.begin(9600);
}

//Read NMEA0183 sentences, when fix is aquired, read the lat/lon and time
//Saves lat/lon and time to EEPROM and RTC clock
bool Gps::Read() {

  bool newdata = false;
  unsigned long start = millis();

  // Every 5 seconds we print an update
  while (millis() - start < 5000) {
    if (Uart.available()) {
      char c = Uart.read();
      Serial.print(c);  // uncomment to see raw GPS data
      if (tgps.encode(c)) {
        newdata = true;
      }
    }
  }

  if (newdata)
  {
    tgps.get_position(&this->latN, &this->lonE, &this->fix_age);
    int Year;
    byte Month, Day, Hour, Minute, Second;
    // retrieves +/- lat/long in 100000ths of a degree
    tgps.get_position(&latN, &lonE, &fix_age);
    // time in hhmmsscc, date in ddmmyy
    tgps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &fix_age);

    if (fix_age == TinyGPS::GPS_INVALID_AGE)
      Serial.println("No fix detected, waiting...");
    else if (fix_age > 5000)
      Serial.println("Warning: possible stale data! waiting...");
    else {
      // set the Time to the latest GPS reading
      delay(500); //delay to avoid writing accidently to EEPROM too many times
      Serial.println("Data is current. Setting up the RTC date and time.");
      setTime(Hour, Minute, Second, Day, Month, Year);
      Serial.println("Saving location to EEPROM.");
      l_Lat2EEPROM(latN);
      l_Lon2EEPROM(lonE);
      serialPrintRTCdateTime();
      Serial.println("Lat: " + String(latN) + " Lon: " + String(lonE));
    }
  }
  else
  {
    Serial.println("No valid data, waiting for GPS...");
  }

  return newdata;
}

//Returns latitude in decimal degrees
long Gps::LatitudeN() {
  return this->latN ;
}

//Returns longitude in decimal degrees
long Gps::LongitudeE() {
  return this->lonE ;
};

#include <TinyGPS.h> //for NMEA0183 GPS data parsing

// Classe utilisée pour communiquer avec le GPS
class Gps
{
  private:
    //Object to parse GPS NMEA0183 data
    TinyGPS tgps;
    //Object to access UART0 connected to the GPS
    HardwareSerial Uart ;
    //latitude/longitude in degrees
    long lat, lon;
    unsigned long fix_age, time, date, speed, course;

    uint32_t lastupdate = 0;
    char raw; // raw GPS data
    bool newdata;

  public:
    void Setup();
    bool Read();
    long Latitude();
    long Longitude();
    unsigned long Date();
    unsigned long Time();
};

void Gps::Setup() {
  HardwareSerial Uart = HardwareSerial();
  Uart.begin(9600);
}

bool Gps::Read() {

  newdata = false;
  
  if (Uart.available()) {
    raw = Uart.read();
    if (tgps.encode(raw)) {
      newdata = true;
    }
  }
  
  if (newdata)
  {
    tgps.get_position(&this->lat, &this->lon, &this->fix_age);
    int Year;
	byte Month, Day, Hour, Minute, Second;
	// retrieves +/- lat/long in 100000ths of a degree
	tgps.get_position(&lat, &lon, &fix_age);
	// time in hhmmsscc, date in ddmmyy
	tgps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &fix_age);
	
	if (fix_age == TinyGPS::GPS_INVALID_AGE)
		Serial.println("No fix detected, waiting...");
	else if (fix_age > 5000)
		Serial.println("Warning: possible stale data! waiting...");
	else{
		// set the Time to the latest GPS reading
		delay(500); //delay to avoid writing accidently to EEPROM too many times
		Serial.println("Data is current. Setting up the RTC date and time.");
		setTime(Hour, Minute, Second, Day, Month, Year);
		Serial.println("Saving location to EEPROM.");
		l_Lat2EEPROM(lat);
		l_Lon2EEPROM(lon);
		serialPrintRTCdateTime();
		Serial.println("Lat: " + String(lat) + " Lon: " + String(lon));
	}
  }
  else
  {
	  Serial.println("No valid data, waiting for GPS...");
  }

  return newdata;
}

long Gps::Latitude() {
  return this->lat ;
}
long Gps::Longitude() {
  return this->lon ;
};
unsigned long Gps::Date() {
  return this->date ;
}
unsigned long Gps::Time() {
  return this->time;
}
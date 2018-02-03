#include <TinyGPS.h> //for NMEA0183 GPS data parsing
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
    unsigned long age;
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
    //Serial.print(raw);  // uncomment to see raw GPS data
    if (tgps.encode(raw)) {
      newdata = true;
    }
  }
  if (newdata)
  {
    tgps.get_position(&this->lat, &this->lon, &this->age);
    // time in hhmmsscc, date in ddmmyy
    tgps.get_datetime(&date, &time, &fix_age);

    if (SerialDebug) {
      Serial.println("");
      Serial.print("Date ");
      Serial.print(date);
      Serial.print(" ");
      Serial.print(time);
      Serial.println("");
      Serial.println("LAT=");
      Serial.println(this->lat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : this->lat, 6);
      Serial.println(" LON=");
      Serial.println(this->lon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : this->lon, 6);
      Serial.println(" SAT=");
      Serial.println(tgps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : tgps.satellites());
      Serial.println(" PREC=");
      Serial.println(tgps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : tgps.hdop());
    }

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


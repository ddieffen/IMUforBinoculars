#include <TinyGPS.h>

//Object to parse GPS NMEA0183 data
TinyGPS gps;
//Object to access UART0 connected to the GPS
HardwareSerial Uart = HardwareSerial();

//latitude/longitude in degrees
long lat, lon;

void setup()  {
  Uart.begin(9600);
  Serial.begin(9600);
}

void loop() {
  bool newdata = false;
  if (Uart.available()) {
    char c = Uart.read();
    Serial.print(c);  // uncomment to see raw GPS data
    if (gps.encode(c)) {
      newdata = true;
    }
  }

  if (newdata)
  {
    unsigned long age;
    gps.get_position(&lat, &lon, &age);
    Serial.println("LAT=");
    Serial.println(lat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lat, 6);
    Serial.println(" LON=");
    Serial.println(lon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lon, 6);
    Serial.println(" SAT=");
    Serial.println(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.println(" PREC=");
    Serial.println(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }

}

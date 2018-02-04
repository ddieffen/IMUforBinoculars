// Classe utilisée pour initialiser et communiquer avec le MPU9250
Mpu9250 mpu9250;
// Classe utilisée pour réaliser les calculs de quaternions et d'attitude
Imu imu;
// Classe utilisée pour communiquer avec le GPS
Gps gps;
// Classe utilisée pour la communication au format LX2000
Astro astro;
// Classe utilisée pour réaliser les calculs de quaternions et d'attitude
CalculVectoriel vect;

// Initialisation des variables et instantiation des classes
void setup() {
  Serial.begin(9600);
  pinMode(myLed, OUTPUT);
  delay(5000); // long delay so that users have time to read message on the serial port after startup
  setSyncProvider(getTeensy3Time);
  Serial.println("Current RTC date:");
  serialPrintRTCdateTime();
  lat = l_EEPROM2Lat();
  lon = l_EEPROM2Lon();
  Serial.println("Current EEPROM position:");
  Serial.println("Lat: " + String(lat) + " Lon: " + String(lon));
  bool localize = false;

  gps.Setup();
  mpu9250.Setup();
  imu.Setup();

  Serial.println("Read from GPS");
  // Recherche signal GPS
  int timeout = 120;
  do {
	Serial.print(String(timeout) + " ");
    localize = gps.Read();
    timeout-=5;
	blinkLed(3, myLed);
  }  while (localize == false && timeout > 0);
  if (localize) {
	  lat = gps.Latitude();
	  lon = gps.Longitude();
	  Serial.println("New Lat/Lon from GPS");
  }
  else {
	  Serial.println("Lat/Lon read from EEPROM");
  }
  Serial.print("LAT=");
  Serial.print(lat);
  Serial.print(" LON=");
  Serial.println(lon);

  astro.Setup(lat, lon);
}

// Boucle principale
void loop() {
  if (mpu9250.IsDataReady()) {
    mpu9250.readAccelData(accelCount);
    mpu9250.readGyroData(gyroCount);
    mpu9250.readMagData(magCount);

    // Calculer l'angle
/*    imu.Compute(
      accelCount[0], accelCount[1],  -accelCount[2],
      gyroCount[0],  gyroCount[1],   -gyroCount[2],
      magCount[1],   magCount[0],    -magCount[2]
    );
*/
    vect.TraiterMesure(
      accelCount[0], accelCount[1], -accelCount[2],
      magCount[1], magCount[0], -magCount[2] // Les axes y et x sont inverses
    );
  }
  delay(100);
  astro.Calc(vect.Azimut(), vect.Altitude());
  
  if (Serial.available() > 0) {
    astro.Communication();
  }
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
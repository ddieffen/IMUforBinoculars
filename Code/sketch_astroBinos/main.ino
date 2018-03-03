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
  latN = l_EEPROM2Lat();
  lonE = l_EEPROM2Lon();
  Serial.println("Current EEPROM position:");
  Serial.println("Lat: " + String(latN) + " Lon: " + String(lonE));
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
    timeout -= 5;
    blinkLed(3, myLed);
  }  while (localize == false && timeout > 0);
  if (localize) {
    latN = gps.LatitudeN();
    lonE = gps.LongitudeE();
    Serial.println("New Lat/Lon from GPS");
  }
  else {
    Serial.println("Lat/Lon read from EEPROM");
  }
  Serial.print("LAT=");
  Serial.print(latN);
  Serial.print(" LON=");
  Serial.println(lonE);

  astro.Setup(latN, lonE);
  SerialDebug = false;
  Serial.println("Initialization complete");
  Serial.println("Begin loop");
}

// Boucle principale
void loop() {
  if (mpu9250.IsDataReady()) {
    mpu9250.readAccelData(accelCount);
    mpu9250.readGyroData(gyroCount);
    mpu9250.readMagData(magData);
    mpu9250.readMagData(magCount);

    // Calculer l'angle
    /*    imu.Compute(
          accelCount[0], accelCount[1],  -accelCount[2],
          gyroCount[0],  gyroCount[1],   -gyroCount[2],
          magCount[1],   magCount[0],    -magCount[2]
        );
    */
    vect.TraiterMesure(
      accelCount[0], -accelCount[1], -accelCount[2],
      magCount[1], magCount[0], magCount[2] // Les axes y et x sont inverses
    );
  }
  if (SerialDebug){
    Mpu9250GetTrace();
    vect.Trace();
    astro.Calc(vect.Azimut(), vect.Pitch());
    astro.Trace();
  }
  delay(100);

  if (Serial.available() > 0) {
    astro.Calc(vect.Azimut(), vect.Pitch());
    astro.Communication();
  }
  if (MagCalibrate) {
    mpu9250.magRecalibrate();
    MagCalibrate = false;
  }
  if (MagCalibRaz) {
    mpu9250.magCalibRaz();
    MagCalibRaz = false;
  }
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

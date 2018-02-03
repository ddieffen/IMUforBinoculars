
// Centrale inertielle
Mpu9250 mpu9250;

Imu imu;

// GPS
Gps gps;

// Communication LX2000 et debogage
Astro astro;

CalculVectoriel vect;


void setup() {
  bool localize = false;
  Serial.begin(9600);
  gps.Setup();
  mpu9250.Setup();
  imu.Setup();
  //  vect.Setup();
  astro.Setup();

  delay(1000);
  Serial.print("Read ");

  // Recherche signal GPS
  do {
    localize = gps.Read();
    //Serial.print("Read ");

  }  while (localize == false);
  lat = gps.Latitude();
  lon = gps.Longitude();
  Serial.print("LAT=");
  Serial.print( lat );
  Serial.print(" LON=");
  Serial.println( lon );

}


void loop() {
  //Serial.println("Attente données");
  if (mpu9250.IsDataReady()) {
    mpu9250.readAccelData(accelCount);
    mpu9250.readGyroData(gyroCount);
    mpu9250.readMagData(magCount);


    // Calculer l'angle
    imu.Compute(
      accelCount[0], accelCount[1],  -accelCount[2],
      gyroCount[0],  gyroCount[1],   -gyroCount[2],
      magCount[1],   magCount[0],    -magCount[2]
    );

    vect.TraiterMesure(
      accelCount[0],
      accelCount[1],
      -accelCount[2],
      magCount[1],
      magCount[0], // Les axes y et x sont inverses
      -magCount[2]
    );
    vect.Trace();
    Serial.println("");
    Serial.print("Pitch = ");
    Serial.print(imu.Pitch());
    Serial.print("Rall = ");
    Serial.print(imu.Roll());
    Serial.print(" ; Yaw = ");
    Serial.println(imu.Yaw());
    /*    Serial.print("Azimut ");
        Serial.println(vect.Azimut());
        Serial.print("Altitude ");
        Serial.println(vect.Altitude());
    */

    Serial.println("");



  }
  delay(100);
  //astro.Communication();

}


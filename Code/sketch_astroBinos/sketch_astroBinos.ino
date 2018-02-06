/* IMUforBinoculars
  by: ddieffen, xgravouil
  date: January, 25 2017
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
*/
#include <TimeLib.h>
#include <TinyGPS.h>
#include <EEPROM.h>

// Set up pin 13 led for toggling
int myLed = 13;
// State of the led pin
int state = 1;

//latitude Nord default latitude and longitude
double latN = 47.9975;
//longitude Est default latitude and longitude
double lonE = -4.0979;
//defaut magnetic declinaison
long magdec = -3;

//if set to true, lots of debugging information will be available on the serial port
bool SerialDebug = true;

float accelCount[3];
float gyroCount[3];
float magCount[3];

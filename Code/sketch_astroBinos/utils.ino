/* Utilities
  by: ddieffen, xgravouil
  date: January, 25 2017
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
*/

//Flashes n times the LED
void blinkLed(int flashes, int ledPin) {
  for (int i = 0; i < flashes; i++) {
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}

//Print � float number in a human readable way
void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) {
    Serial.print('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print(".");

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}

//Uses TimeLib RTC functions to display time in a human readable way
void serialPrintRTCdateTime() {
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}

//Utility function for digital clock display: prints preceding colon and leading 0
void printDigits(int digits) {
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

//Stores latitude to EEPROM addr 0
void l_Lat2EEPROM(long lat) {
  l_EEPROMwrite(lat, 0);
}

//Stores latitude to EEPROM addr 4
void l_Lon2EEPROM(long lon) {
  l_EEPROMwrite(lon, 4);
}

//Reads latitude from EEPROM addr 0
long l_EEPROM2Lat() {
  return l_EEPROMread(0);
}

//Reads latitude from EEPROM addr 4
long l_EEPROM2Lon() {
  return l_EEPROMread(4);
}

//Wrties a long to EEPROM at a specific address by splitting it in 4 bytes
void l_EEPROMwrite(long value, int address) {
  byte lbuffer[4];
  lbuffer[0] = value >> 24;
  lbuffer[1] = value >> 16;
  lbuffer[2] = value >> 8;
  lbuffer[3] = value;
  EEPROM.write(address, lbuffer[0]);
  EEPROM.write(address + 1, lbuffer[1]);
  EEPROM.write(address + 2, lbuffer[2]);
  EEPROM.write(address + 3, lbuffer[3]);
}

//Reads a long from EEPROM at a specific address by splitting it in 4 bytes
long l_EEPROMread(int address) {
  byte lbuffer[4];
  lbuffer[0] = EEPROM.read(address);
  lbuffer[1] = EEPROM.read(address + 1);
  lbuffer[2] = EEPROM.read(address + 2);
  lbuffer[3] = EEPROM.read(address + 3);
  long recomb = lbuffer[0] << 24 | lbuffer[1] << 16 | lbuffer[2] << 8 | lbuffer[3];
  return recomb;
}



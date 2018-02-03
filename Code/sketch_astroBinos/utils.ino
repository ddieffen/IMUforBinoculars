void blinkLed(int flashes, int ledPin) {
  for (int i = 0; i < flashes; i++) {
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(200);
  }
}
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
void digitalClockDisplay() {
  // digital clock display of the time
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

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void l_Lat2EEPROM(long lat) {
  //Serial.println("Storing lat: " + String(lat));
  l_EEPROMwrite(lat, 0);
}

void l_Lon2EEPROM(long lon) {
  //Serial.println("Storing lon: " + String(lon));
  l_EEPROMwrite(lon, 4);
}

long l_EEPROM2Lat() {
  //Serial.println("Reading lat at EEPROM address 0");
  return l_EEPROMread(0);
}

long l_EEPROM2Lon() {
  //Serial.println("Reading lat at EEPROM address 4");
  return l_EEPROMread(4);
}

void l_EEPROMwrite(long value, int address) {
  byte lbuffer[4];
  lbuffer[0] = value >> 24;
  lbuffer[1] = value >> 16;
  lbuffer[2] = value >> 8;
  lbuffer[3] = value;
  //Serial.println("Storing byte " + String(lbuffer[0]) + " at address " + String(address));
  EEPROM.write(address, lbuffer[0]);
  //Serial.println("Storing byte " + String(lbuffer[1]) + " at address " + String(address+1));
  EEPROM.write(address + 1, lbuffer[1]);
  //Serial.println("Storing byte " + String(lbuffer[2]) + " at address " + String(address+2));
  EEPROM.write(address + 2, lbuffer[2]);
  //Serial.println("Storing byte " + String(lbuffer[3]) + " at address " + String(address+3));
  EEPROM.write(address + 3, lbuffer[3]);
}

long l_EEPROMread(int address) {
  byte lbuffer[4];
  //Serial.print("Reading byte at adress " + String(address));
  lbuffer[0] = EEPROM.read(address);
  //Serial.println(" returns value " + String(lbuffer[0]));
  //Serial.print("Reading byte at adress " + String(address+1));
  lbuffer[1] = EEPROM.read(address + 1);
  //Serial.println(" returns value " + String(lbuffer[1]));
  //Serial.print("Reading byte at adress " + String(address+2));
  lbuffer[2] = EEPROM.read(address + 2);
  //Serial.println(" returns value " + String(lbuffer[2]));
  //Serial.print("Reading byte at adress " + String(address+3));
  lbuffer[3] = EEPROM.read(address + 3);
  //Serial.println(" returns value " + String(lbuffer[3]));
  long recomb = lbuffer[0] << 24 | lbuffer[1] << 16 | lbuffer[2] << 8 | lbuffer[3];
  return recomb;
}



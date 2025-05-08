#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define EEPROM_ADDR 0x50
bool testEEPROM();
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nEEPROM I2C Basic Test");
  
  // I2C initialisieren
  Wire.begin(SDA_PIN, SCL_PIN);
  // Langsame Geschwindigkeit für bessere Stabilität
  Wire.setClock(100000);
  
  // I2C-Scanner
  Serial.println("I2C-Scanner läuft...");
  byte error, address;
  int deviceCount = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C-Gerät gefunden auf Adresse 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Überprüfen, ob es unser EEPROM ist
      if (address == EEPROM_ADDR) {
        Serial.println(" <-- EEPROM!");
      } else {
        Serial.println();
      }
      
      deviceCount++;
      delay(10);
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("Keine I2C-Geräte gefunden");
    return;
  }
  
  // EEPROM-Test
  if (testEEPROM()) {
    Serial.println("EEPROM-Test erfolgreich!");
  } else {
    Serial.println("EEPROM-Test fehlgeschlagen!");
  }
}

bool testEEPROM() {
  // Schreibe Testdaten (Adresse 0)
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(0);    // Adresse MSB
  Wire.write(0);    // Adresse LSB
  Wire.write(0xAA); // Testdaten
  if (Wire.endTransmission() != 0) {
    Serial.println("Fehler beim Schreiben");
    return false;
  }
  
  // Warte auf den Schreibvorgang
  delay(10);
  
  // Setze Leseadresse
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(0);    // Adresse MSB
  Wire.write(0);    // Adresse LSB
  if (Wire.endTransmission() != 0) {
    Serial.println("Fehler beim Setzen der Leseadresse");
    return false;
  }
  
  // Lese Daten
  Wire.requestFrom(EEPROM_ADDR, 1);
  if (Wire.available()) {
    byte data = Wire.read();
    Serial.print("Gelesene Daten: 0x");
    Serial.println(data, HEX);
    return (data == 0xAA);
  }
  
  Serial.println("Keine Daten vom EEPROM erhalten");
  return false;
}

void loop() {
  delay(1000);
}
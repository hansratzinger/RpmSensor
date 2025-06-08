#include <Arduino.h>
#include <Wire.h>

// I2C Pins definieren
#define SDA_PIN 16
#define SCL_PIN 17

// Verschiedene mögliche EEPROM-Adressen
const byte possibleAddresses[] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57};

// Test-Funktion für einen einzelnen EEPROM-Schreib- und Lesevorgang
bool testEEPROMAtAddress(byte address, uint16_t testLocation, byte testValue) {
  Serial.print("Test EEPROM bei Adresse 0x");
  Serial.print(address, HEX);
  Serial.print(" mit Wert 0x");
  Serial.print(testValue, HEX);
  Serial.print(" an Position 0x");
  Serial.print(testLocation, HEX);
  Serial.println("...");
  
  // Schreibvorgang
  Wire.beginTransmission(address);
  Wire.write((testLocation >> 8) & 0xFF);  // MSB
  Wire.write(testLocation & 0xFF);         // LSB
  Wire.write(testValue);                   // Testwert
  byte writeResult = Wire.endTransmission();
  
  Serial.print("  Schreibergebnis: ");
  Serial.print(writeResult);
  Serial.println(writeResult == 0 ? " (OK)" : " (FEHLER)");
  
  if (writeResult != 0) return false;
  
  delay(10); // EEPROM-Schreibzyklus abwarten
  
  // Lesevorgang
  Wire.beginTransmission(address);
  Wire.write((testLocation >> 8) & 0xFF);  // MSB
  Wire.write(testLocation & 0xFF);         // LSB
  byte setAddrResult = Wire.endTransmission();
  
  Serial.print("  Leseadresse setzen: ");
  Serial.print(setAddrResult);
  Serial.println(setAddrResult == 0 ? " (OK)" : " (FEHLER)");
  
  if (setAddrResult != 0) return false;
  
  // Anfrage für ein Byte
  byte requestResult = Wire.requestFrom(address, (uint8_t)1);
  
  Serial.print("  Leseanfrage-Ergebnis: ");
  Serial.print(requestResult);
  Serial.println(requestResult == 1 ? " (OK)" : " (FEHLER)");
  
  if (requestResult != 1) return false;
  
  // Gelesenes Byte
  byte readValue = Wire.read();
  
  Serial.print("  Gelesener Wert: 0x");
  Serial.println(readValue, HEX);
  
  return (readValue == testValue);
}

// I2C-Bus vollständig zurücksetzen
void resetI2CBus() {
  Serial.println("I2C-Bus zurücksetzen...");
  
  // I2C-Bus komplett deaktivieren
  Wire.end();
  delay(300);
  
  // Pins als Eingänge mit Pullup konfigurieren
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(100);
  
  // Pins als Ausgänge konfigurieren
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  
  // Beide Leitungen auf HIGH (Idle-Zustand)
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
  delay(100);
  
  // Clock-Cycling: 16 Taktzyklen senden
  for (int i = 0; i < 16; i++) {
    digitalWrite(SCL_PIN, LOW);
    delay(5);
    digitalWrite(SCL_PIN, HIGH);
    delay(5);
  }
  
  // STOP-Bedingung simulieren
  digitalWrite(SDA_PIN, LOW);
  delay(5);
  digitalWrite(SCL_PIN, HIGH);
  delay(5);
  digitalWrite(SDA_PIN, HIGH);
  delay(20);
  
  Serial.println("I2C-Bus Reset abgeschlossen.");
}

// Testfunktion mit verschiedenen Frequenzen
void testWithFrequency(uint32_t frequency) {
  Serial.print("\n>>> TESTE MIT I2C-FREQUENZ: ");
  Serial.print(frequency / 1000);
  Serial.println(" kHz <<<\n");
  
  Wire.setClock(frequency);
  delay(100);
  
  // I2C-Adress-Scan
  Serial.println("I2C-Scanner (suche alle Geräte):");
  Serial.println("-----------------------------");
  byte deviceCount = 0;
  
  for (byte address = 1; address < 128; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Gerät gefunden an Adresse 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }
  
  Serial.print("Insgesamt ");
  Serial.print(deviceCount);
  Serial.println(" Geräte gefunden\n");
  
  // EEPROM-Test an jeder möglichen Adresse
  Serial.println("EEPROM-Tests an verschiedenen Adressen:");
  Serial.println("-------------------------------------");
  
  for (int i = 0; i < sizeof(possibleAddresses); i++) {
    // Basis-Erreichbarkeitstest
    Wire.beginTransmission(possibleAddresses[i]);
    byte error = Wire.endTransmission();
    
    Serial.print("Adresse 0x");
    Serial.print(possibleAddresses[i], HEX);
    Serial.print(": ");
    
    if (error == 0) {
      Serial.println("GERÄT GEFUNDEN - Führe Lese/Schreib-Tests durch");
      
      // Test an mehreren Positionen
      bool test1 = testEEPROMAtAddress(possibleAddresses[i], 0x0000, 0xA5);
      delay(50);
      bool test2 = testEEPROMAtAddress(possibleAddresses[i], 0x0001, 0x5A);
      delay(50);
      bool test3 = testEEPROMAtAddress(possibleAddresses[i], 0x0010, 0xF0);
      
      Serial.println();
      Serial.print("  Testergebnisse für 0x");
      Serial.print(possibleAddresses[i], HEX);
      Serial.print(": ");
      
      if (test1 && test2 && test3) {
        Serial.println("ALLE TESTS BESTANDEN - EEPROM FUNKTIONIERT!");
      } else {
        Serial.print("Tests fehlgeschlagen (");
        Serial.print(test1 ? "OK" : "FAIL");
        Serial.print(", ");
        Serial.print(test2 ? "OK" : "FAIL");
        Serial.print(", ");
        Serial.print(test3 ? "OK" : "FAIL");
        Serial.println(")");
      }
    } else {
      Serial.print("Nicht gefunden (Fehlercode: ");
      Serial.print(error);
      Serial.println(")");
    }
    
    Serial.println(); // Leerzeile für bessere Lesbarkeit
  }
}
// Code in deinem Arduino einfügen, um gezielt eine I2C-Transaktion auszulösen
void triggerSingleI2CTransaction() {
  Serial.println("Single I2C test starting in 4 seconds...");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000); 
  Serial.println("1");
  delay(1000);
  Serial.println("NOW!");
  
  // // Speichere den aktuellen I2C-Status
  // Wire.beginTransmission(0x50);  // EEPROM-Adresse
  // Wire.write(0);                 // Adresse MSB
  // Wire.write(0);                 // Adresse LSB
  // Wire.write(0xAA);              // Testmuster
  // Wire.endTransmission();
  
// Im triggerSingleI2CTransaction() für jeden Test eine andere Adresse
for (byte addr = 0x50; addr <= 0x57; addr++) {
  Serial.printf("Testing address 0x%02X\n", addr);
  Wire.beginTransmission(addr);
  Wire.write(0);
  Wire.write(0);
  Wire.write(0xAA);
  byte result = Wire.endTransmission();
  Serial.printf("Result: %d\n", result);
  delay(100);
}

  Serial.println("I2C test completed");
  delay(5000);  // Pause vor dem nächsten Test
}
void setup() {
  Serial.begin(115200);
  delay(2000); // Warten, bis Serial Monitor geöffnet wird
  
  Serial.println("\n\n=================================================");
  Serial.println("       ESP32 EEPROM DIAGNOSE-PROGRAMM");
  Serial.println("=================================================\n");
  
  Serial.println("Dieses Programm testet die Kommunikation mit einem externen");
  Serial.println("EEPROM-Chip an verschiedenen I2C-Adressen mit mehreren");
  Serial.println("Geschwindigkeiten, um Hardwareprobleme zu isolieren.\n");
  
  Serial.println("Systeminfo:");
  Serial.print("- ESP32 SDK Version: ");
  Serial.println(ESP.getSdkVersion());
  Serial.print("- Freier Heap: ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("- SDA-Pin: ");
  Serial.println(SDA_PIN);
  Serial.print("- SCL-Pin: ");
  Serial.println(SCL_PIN);
  
  Serial.println("\nTest wird gestartet...");
  
  // I2C initialisieren
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  
  // I2C-Bus zurücksetzen
  resetI2CBus();
  
  // I2C-Bus neu starten
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(200);
  
  // Tests mit verschiedenen Geschwindigkeiten
  testWithFrequency(10000);   // 10 kHz (extrem langsam)
  resetI2CBus();
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(200);
  
  testWithFrequency(50000);   // 50 kHz (langsam)
  resetI2CBus();
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(200);
  
  testWithFrequency(100000);  // 100 kHz (Standard)
  
  Serial.println("\n=================================================");
  Serial.println("         TESTS ABGESCHLOSSEN");
  Serial.println("=================================================\n");
  
  Serial.println("Bitte überprüfen Sie die Ausgaben auf Probleme und");
  Serial.println("stellen Sie sicher, dass die Hardware korrekt angeschlossen ist:");
  Serial.println("1. Sind die Pullup-Widerstände angeschlossen?");
  Serial.println("2. Sind die EEPROM-Adresspins A0, A1, A2 korrekt gesetzt?");
  Serial.println("3. Ist die Stromversorgung des EEPROMs stabil bei 3,3V?");
  Serial.println("4. Ist der WP-Pin (Write Protect) auf GND gesetzt?");
  
  Serial.println("\nProgramm verbleibt in Dauerschleife.");
}

void loop() {
  delay(10000); // Nichts zu tun in der Hauptschleife
  Serial.println("Programm läuft...");
  delay(1000);
    // Frequenz vor dem Test ändern (z.B. auf 10 kHz für bessere Beobachtung)
  Wire.setClock(10000);
  triggerSingleI2CTransaction(); // Einmalige I2C-Transaktion auslösen
  delay(10000); // Wartezeit zwischen den Transaktionen
}
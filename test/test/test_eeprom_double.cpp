#include <Arduino.h>
#include <Wire.h>

// I2C Pins definieren
#define SDA_PIN_EEPROM 16  // EEPROM-Bus
#define SCL_PIN_EEPROM 17  // EEPROM-Bus
#define SDA_PIN_OLED 21    // OLED/RTC-Bus
#define SCL_PIN_OLED 22    // OLED/RTC-Bus

// Verschiedene mögliche EEPROM-Adressen
const byte possibleAddresses[] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57};

// Mögliche OLED/RTC-Adressen
const byte oledAddresses[] = {0x3C, 0x3D}; // Typische OLED-Adressen
const byte rtcAddress = 0x68;              // DS3231 RTC-Adresse


// Test-Funktion für einen einzelnen EEPROM-Schreib- und Lesevorgang
bool testEEPROMAtAddress(TwoWire &wirePort, byte address, uint16_t testLocation, byte testValue) {
  Serial.print("Test EEPROM bei Adresse 0x");
  Serial.print(address, HEX);
  Serial.print(" mit Wert 0x");
  Serial.print(testValue, HEX);
  Serial.print(" an Position 0x");
  Serial.print(testLocation, HEX);
  Serial.println("...");
  
  // Schreibvorgang
  wirePort.beginTransmission(address);
  wirePort.write((testLocation >> 8) & 0xFF);  // MSB
  wirePort.write(testLocation & 0xFF);         // LSB
  wirePort.write(testValue);                   // Testwert
  byte writeResult = wirePort.endTransmission();
  
  Serial.print("  Schreibergebnis: ");
  Serial.print(writeResult);
  Serial.println(writeResult == 0 ? " (OK)" : " (FEHLER)");
  
  if (writeResult != 0) return false;
  
  delay(10); // EEPROM-Schreibzyklus abwarten
  
  // Lesevorgang
  wirePort.beginTransmission(address);
  wirePort.write((testLocation >> 8) & 0xFF);  // MSB
  wirePort.write(testLocation & 0xFF);         // LSB
  byte setAddrResult = wirePort.endTransmission();
  
  Serial.print("  Leseadresse setzen: ");
  Serial.print(setAddrResult);
  Serial.println(setAddrResult == 0 ? " (OK)" : " (FEHLER)");
  
  if (setAddrResult != 0) return false;
  
  // Anfrage für ein Byte
  byte requestResult = wirePort.requestFrom(address, (uint8_t)1);
  
  Serial.print("  Leseanfrage-Ergebnis: ");
  Serial.print(requestResult);
  Serial.println(requestResult == 1 ? " (OK)" : " (FEHLER)");
  
  if (requestResult != 1) return false;
  
  // Gelesenes Byte
  byte readValue = wirePort.read();
  
  Serial.print("  Gelesener Wert: 0x");
  Serial.println(readValue, HEX);
  
  return (readValue == testValue);
}

// I2C-Bus vollständig zurücksetzen
void resetI2CBus(int sdaPin, int sclPin) {
  Serial.println("I2C-Bus zurücksetzen...");
  
  // I2C-Bus komplett deaktivieren
  if (sdaPin == SDA_PIN_EEPROM) {
    Wire.end();
  } else {
    Wire1.end();
  }
  delay(300);
  
  // Pins als Eingänge mit Pullup konfigurieren
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, INPUT_PULLUP);
  delay(100);
  
  // Pins als Ausgänge konfigurieren
  pinMode(sdaPin, OUTPUT);
  pinMode(sclPin, OUTPUT);
  
  // Beide Leitungen auf HIGH (Idle-Zustand)
  digitalWrite(sdaPin, HIGH);
  digitalWrite(sclPin, HIGH);
  delay(100);
  
  // Clock-Cycling: 16 Taktzyklen senden
  for (int i = 0; i < 16; i++) {
    digitalWrite(sclPin, LOW);
    delay(5);
    digitalWrite(sclPin, HIGH);
    delay(5);
  }
  
  // STOP-Bedingung simulieren
  digitalWrite(sdaPin, LOW);
  delay(5);
  digitalWrite(sclPin, HIGH);
  delay(5);
  digitalWrite(sdaPin, HIGH);
  delay(20);
  
  Serial.println("I2C-Bus Reset abgeschlossen.");
}

// Testfunktion mit verschiedenen Frequenzen
void testWithFrequency(TwoWire &wirePort, int sdaPin, int sclPin, uint32_t frequency) {
  Serial.print("\n>>> TESTE MIT I2C-FREQUENZ: ");
  Serial.print(frequency / 1000);
  Serial.print(" kHz auf Bus ");
  Serial.print((sdaPin == SDA_PIN_EEPROM) ? "EEPROM (16/17)" : "OLED/RTC (21/22)");
  Serial.println(" <<<\n");
  
  wirePort.setClock(frequency);
  delay(100);
  
  // I2C-Adress-Scan
  Serial.println("I2C-Scanner (suche alle Geräte):");
  Serial.println("-----------------------------");
  byte deviceCount = 0;
  
  for (byte address = 1; address < 128; address++) {
    wirePort.beginTransmission(address);
    byte error = wirePort.endTransmission();
    
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
  
  // Wenn EEPROM-Bus, dann EEPROM-Tests durchführen
  if (sdaPin == SDA_PIN_EEPROM) {
    // EEPROM-Test an jeder möglichen Adresse
    Serial.println("EEPROM-Tests an verschiedenen Adressen:");
    Serial.println("-------------------------------------");
    
    for (int i = 0; i < sizeof(possibleAddresses); i++) {
      // Basis-Erreichbarkeitstest
      wirePort.beginTransmission(possibleAddresses[i]);
      byte error = wirePort.endTransmission();
      
      Serial.print("Adresse 0x");
      Serial.print(possibleAddresses[i], HEX);
      Serial.print(": ");
      
      if (error == 0) {
        Serial.println("GERÄT GEFUNDEN - Führe Lese/Schreib-Tests durch");
        
        // Test an mehreren Positionen
        bool test1 = testEEPROMAtAddress(wirePort, possibleAddresses[i], 0x0000, 0xA5);
        delay(50);
        bool test2 = testEEPROMAtAddress(wirePort, possibleAddresses[i], 0x0001, 0x5A);
        delay(50);
        bool test3 = testEEPROMAtAddress(wirePort, possibleAddresses[i], 0x0010, 0xF0);
        
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
  else { // OLED/RTC-Bus Tests
    Serial.println("OLED/RTC-Bus Gerätetest:");
    Serial.println("----------------------");
    
    // OLED-Adressen testen
    for (int i = 0; i < sizeof(oledAddresses); i++) {
      wirePort.beginTransmission(oledAddresses[i]);
      byte error = wirePort.endTransmission();
      
      Serial.print("OLED-Adresse 0x");
      Serial.print(oledAddresses[i], HEX);
      Serial.print(": ");
      
      if (error == 0) {
        Serial.println("GERÄT GEFUNDEN");
      } else {
        Serial.print("Nicht gefunden (Fehlercode: ");
        Serial.print(error);
        Serial.println(")");
      }
    }
    
    // RTC-Adresse testen
    wirePort.beginTransmission(rtcAddress);
    byte error = wirePort.endTransmission();
    
    Serial.print("RTC-Adresse 0x");
    Serial.print(rtcAddress, HEX);
    Serial.print(": ");
    
    if (error == 0) {
      Serial.println("GERÄT GEFUNDEN");
      
      // RTC-Daten lesen
      wirePort.beginTransmission(rtcAddress);
      wirePort.write(0x00); // Starte bei Register 0
      wirePort.endTransmission();
      
      wirePort.requestFrom(rtcAddress, (uint8_t)7); // Lese 7 Bytes (Sekunden, Minuten, Stunden, Tag, Datum, Monat, Jahr)
      
      if (wirePort.available() >= 7) {
        byte second = wirePort.read() & 0x7F;
        byte minute = wirePort.read() & 0x7F;
        byte hour = wirePort.read() & 0x3F;
        byte dayOfWeek = wirePort.read();
        byte day = wirePort.read();
        byte month = wirePort.read();
        byte year = wirePort.read();
        
        // BCD zu Dezimal konvertieren
        second = (second >> 4) * 10 + (second & 0x0F);
        minute = (minute >> 4) * 10 + (minute & 0x0F);
        hour = (hour >> 4) * 10 + (hour & 0x0F);
        day = (day >> 4) * 10 + (day & 0x0F);
        month = (month >> 4) * 10 + (month & 0x0F);
        year = (year >> 4) * 10 + (year & 0x0F);
        
        Serial.print("  RTC-Zeit: 20");
        Serial.print(year);
        Serial.print("-");
        Serial.print(month);
        Serial.print("-");
        Serial.print(day);
        Serial.print(" ");
        Serial.print(hour);
        Serial.print(":");
        Serial.print(minute);
        Serial.print(":");
        Serial.println(second);
      }
    } else {
      Serial.print("Nicht gefunden (Fehlercode: ");
      Serial.print(error);
      Serial.println(")");
    }
    
    Serial.println(); // Leerzeile für bessere Lesbarkeit
  }
}

// Code um gezielt eine I2C-Transaktion auszulösen
void triggerSingleI2CTransaction(TwoWire &wirePort, int busType) {
  Serial.print("Single I2C test starting on ");
  Serial.print(busType == 0 ? "EEPROM Bus (16/17)" : "OLED/RTC Bus (21/22)");
  Serial.println(" in 4 seconds...");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000); 
  Serial.println("1");
  delay(1000);
  Serial.println("NOW!");
  
  if (busType == 0) { // EEPROM Bus
    // Test EEPROM-Adressen
    for (byte addr = 0x50; addr <= 0x57; addr++) {
      Serial.printf("Testing EEPROM address 0x%02X\n", addr);
      wirePort.beginTransmission(addr);
      wirePort.write(0);
      wirePort.write(0);
      wirePort.write(0xAA);
      byte result = wirePort.endTransmission();
      Serial.printf("Result: %d\n", result);
      delay(100);
    }
  } else { // OLED/RTC Bus
    // Test OLED-Adressen
    for (int i = 0; i < sizeof(oledAddresses); i++) {
      Serial.printf("Testing OLED address 0x%02X\n", oledAddresses[i]);
      wirePort.beginTransmission(oledAddresses[i]);
      byte result = wirePort.endTransmission();
      Serial.printf("Result: %d\n", result);
      delay(100);
    }
    
    // Test RTC-Adresse
    Serial.printf("Testing RTC address 0x%02X\n", rtcAddress);
    wirePort.beginTransmission(rtcAddress);
    byte result = wirePort.endTransmission();
    Serial.printf("Result: %d\n", result);
    delay(100);
  }

  Serial.println("I2C test completed");
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  delay(2000); // Warten, bis Serial Monitor geöffnet wird
  
  Serial.println("\n\n=================================================");
  Serial.println("       ESP32 I2C-BUSSE DIAGNOSE-PROGRAMM");
  Serial.println("=================================================\n");
  
  Serial.println("Dieses Programm testet die Kommunikation mit Geräten");
  Serial.println("an zwei verschiedenen I2C-Bussen mit mehreren");
  Serial.println("Geschwindigkeiten, um Hardwareprobleme zu isolieren.\n");
  
  Serial.println("Systeminfo:");
  Serial.print("- ESP32 SDK Version: ");
  Serial.println(ESP.getSdkVersion());
  Serial.print("- Freier Heap: ");
  Serial.println(ESP.getFreeHeap());
  Serial.println("- Bus 1 (EEPROM):");
  Serial.print("  SDA-Pin: ");
  Serial.println(SDA_PIN_EEPROM);
  Serial.print("  SCL-Pin: ");
  Serial.println(SCL_PIN_EEPROM);
  Serial.println("- Bus 2 (OLED/RTC):");
  Serial.print("  SDA-Pin: ");
  Serial.println(SDA_PIN_OLED);
  Serial.print("  SCL-Pin: ");
  Serial.println(SCL_PIN_OLED);
  
  // ================================================================
  // Test EEPROM-Bus (I2C Bus 1 auf Pins 16/17)
  // ================================================================
  Serial.println("\n\n=================================================");
  Serial.println("       TEST EEPROM-BUS (Pins 16/17)");
  Serial.println("=================================================\n");
  
  Serial.println("\nEEPROM-Bus-Test wird gestartet...");
  
  // I2C initialisieren
  Wire.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  Wire.setClock(100000);
  
  // I2C-Bus zurücksetzen
  resetI2CBus(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  
  // I2C-Bus neu starten
  Wire.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  delay(200);
  
  // Tests mit verschiedenen Geschwindigkeiten
  testWithFrequency(Wire, SDA_PIN_EEPROM, SCL_PIN_EEPROM, 10000);   // 10 kHz
  resetI2CBus(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  Wire.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  delay(200);
  
  testWithFrequency(Wire, SDA_PIN_EEPROM, SCL_PIN_EEPROM, 50000);   // 50 kHz
  resetI2CBus(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  Wire.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  delay(200);
  
  testWithFrequency(Wire, SDA_PIN_EEPROM, SCL_PIN_EEPROM, 100000);  // 100 kHz
  
  // ================================================================
  // Test OLED/RTC-Bus (I2C Bus 2 auf Pins 21/22)
  // ================================================================
  Serial.println("\n\n=================================================");
  Serial.println("       TEST OLED/RTC-BUS (Pins 21/22)");
  Serial.println("=================================================\n");
  
  Serial.println("\nOLED/RTC-Bus-Test wird gestartet...");
  
  // I2C initialisieren
  Wire1.begin(SDA_PIN_OLED, SCL_PIN_OLED);
  Wire1.setClock(100000);
  
  // I2C-Bus zurücksetzen
  resetI2CBus(SDA_PIN_OLED, SCL_PIN_OLED);
  
  // I2C-Bus neu starten
  Wire1.begin(SDA_PIN_OLED, SCL_PIN_OLED);
  delay(200);
  
  // Tests mit verschiedenen Geschwindigkeiten
  testWithFrequency(Wire1, SDA_PIN_OLED, SCL_PIN_OLED, 10000);   // 10 kHz
  resetI2CBus(SDA_PIN_OLED, SCL_PIN_OLED);
  Wire1.begin(SDA_PIN_OLED, SCL_PIN_OLED);
  delay(200);
  
  testWithFrequency(Wire1, SDA_PIN_OLED, SCL_PIN_OLED, 50000);   // 50 kHz
  resetI2CBus(SDA_PIN_OLED, SCL_PIN_OLED);
  Wire1.begin(SDA_PIN_OLED, SCL_PIN_OLED);
  delay(200);
  
  testWithFrequency(Wire1, SDA_PIN_OLED, SCL_PIN_OLED, 100000);  // 100 kHz
  
  // ================================================================
  // Zusammenfassung
  // ================================================================
  Serial.println("\n=================================================");
  Serial.println("         ALLE TESTS ABGESCHLOSSEN");
  Serial.println("=================================================\n");
  
  Serial.println("Bitte überprüfen Sie die Ausgaben auf Probleme und");
  Serial.println("stellen Sie sicher, dass die Hardware korrekt angeschlossen ist.\n");
  
  Serial.println("EEPROM-Bus (16/17):");
  Serial.println("1. Sind die Pullup-Widerstände angeschlossen?");
  Serial.println("2. Sind die EEPROM-Adresspins A0, A1, A2 korrekt gesetzt?");
  Serial.println("3. Ist die Stromversorgung des EEPROMs stabil bei 3,3V?");
  Serial.println("4. Ist der WP-Pin (Write Protect) auf GND gesetzt?");
  
  Serial.println("\nOLED/RTC-Bus (21/22):");
  Serial.println("1. Sind die Pullup-Widerstände angeschlossen?");
  Serial.println("2. Sind die OLED-Displays an den korrekten Adressen (0x3C, 0x3D)?");
  Serial.println("3. Ist die RTC an der Adresse 0x68 erreichbar?");
  
  Serial.println("\nProgramm verbleibt in Dauerschleife.");
}

void loop() {
  delay(5000); // Pause zwischen den Tests
  
  // EEPROM-Bus testen
  Serial.println("\n--- Periodischer EEPROM-Bus Test ---");
  Wire.setClock(10000);
  triggerSingleI2CTransaction(Wire, 0);
  
  delay(5000); // Pause zwischen den Tests
  
  // OLED/RTC-Bus testen
  Serial.println("\n--- Periodischer OLED/RTC-Bus Test ---");
  Wire1.setClock(10000);
  triggerSingleI2CTransaction(Wire1, 1);
}
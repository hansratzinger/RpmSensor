// -------------------------------------------------
// RpmSensorRtcOled.cpp
// -------------------------------------------------
// Usage for measuring RMP (Revolutions per Minute) with an infrared sensor
// and displaying the result on OLED displays. The DS3231 RTC is used to keep track of time.
// The SD card is used to log the data. The program allows setting the RTC time using buttons on the ESP32 board.
// ESP32 DS3231 RTC + Two OLED Displays + Infrared Sensor
// RELEASE 3.0 HR 2025-04-25 NK
// -------------------------------------------------
// Changed from single OLED to dual OLED displays (RPM and Time)
// -------------------------------------------------
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.
// --------------------------------------------------
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>

#define SD_CS 5 // SD-Karte Chip Select Pin

// Pin-Definitionen
#define BUTTON_PLUS 32  // GPIO für die "+"-Taste
#define BUTTON_MINUS 33 // GPIO für die "-"-Taste
#define BUTTON_SET 27   // GPIO für die "SET"-Taste

// Neue Konstante für genauere RPM-Messung
#define RpmTriggerPerRound 2 // 2 Impulse pro Umdrehung für präzisere Messung

#define IR_SENSOR_PIN 15 // GPIO15: Infrarotsensor Pin
#define LED_PIN 12      // GPIO12: Kontroll-LED Pin

// I2C Pins für einen gemeinsamen Bus
#define SDA_PIN 21
#define SCL_PIN 22

// OLED Display Adressen (7-Bit Format)
#define OLED1_ADDR 0x3D // 0x7A in 8-Bit-Format (RPM-Display)
#define OLED2_ADDR 0x3C // 0x78 in 8-Bit-Format (Zeit-Display)

// Initialisierung der OLED-Displays am gleichen Bus
U8G2_SSD1306_128X64_NONAME_F_HW_I2C displayRPM(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C displayTime(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Globale Statusvariable für SD-Karte
bool sdCardAvailable = false;

// Globale Variable für den aktuellen Dateinamen
String currentLogFileName;

// RTC Modul
RTC_DS3231 rtc;

// Zustandsvariablen
bool TEST = false; // Debug-Modus
#define CLOCK_INTERRUPT_PIN 14

// Zustandsarten für die RTC-Einstellung
enum SetupState {
  NORMAL,
  SET_DAY,     // Tag einstellen
  SET_MONTH,   // Monat einstellen
  SET_YEAR,    // Jahr einstellen
  SET_HOUR,    // Stunden einstellen
  SET_MINUTE,  // Minuten einstellen
  SET_SECOND,  // Sekunden einstellen
  COMPLETE     // Setup abgeschlossen
};

volatile SetupState currentState = NORMAL;
volatile bool stateChanged = false;
volatile bool buttonPressed = false;

// Variablen für Datum und Zeit
int hour = 0;
int minute = 0;
int second = 0;
int day = 1;     // Tag
int month = 5;   // Monat
int year = 2025; // Jahr

// RPM-Messung
volatile unsigned long Rpm_Count; // Zähler für Interrupts
volatile unsigned long Rpm_Count_LastSecond; // Zähler für letzte Sekunde
int Rpm;                          // Variable für die aktuelle RPM
unsigned long lastTime;           // Variable für die letzte Zeitmessung
unsigned long lastOutputTime = 0; // Zeitpunkt der letzten Ausgabe
unsigned long lastSecondRpmCount = 0; // Zeitpunkt der letzten Sekundenmessung

// Entprellzeit für den IR-Sensor (in Mikrosekunden)
const unsigned long IR_DEBOUNCE_TIME = 5000; // 5ms Entprellzeit
volatile unsigned long lastIrInterruptTime = 0;

void IRAM_ATTR Rpm_isr() {
  unsigned long interruptTime = micros(); // Nutze Mikrosekunden für präzisere Messung
  
  // Nur zählen, wenn genügend Zeit seit dem letzten Interrupt vergangen ist
  if (interruptTime - lastIrInterruptTime > IR_DEBOUNCE_TIME) {
    Rpm_Count++;
    lastIrInterruptTime = interruptTime;
  }
}

// Variable für die Entprellung hinzufügen
unsigned long lastInterruptTime = 0;
const unsigned long debounceTime = 500; // 500ms Entprellzeit

void IRAM_ATTR handleSetButtonInterrupt() {
  unsigned long interruptTime = millis();
  
  // Nur bei genügend Abstand zum letzten Interrupt
  if (interruptTime - lastInterruptTime > debounceTime) {
    buttonPressed = true;
    lastInterruptTime = interruptTime;
  }
}

// Hilfsfunktion für I2C-Scanner
void scanI2C() {
  byte error, address;
  int deviceCount = 0;
  
  Serial.println("I2C-Scanner gestartet");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C-Gerät gefunden auf Adresse 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("Keine I2C-Geräte gefunden!");
  } else {
    Serial.print("Insgesamt ");
    Serial.print(deviceCount);
    Serial.println(" I2C-Geräte gefunden");
  }
}

// Funktion zur Aktualisierung der Zeit vom RTC mit Debug-Ausgaben
void updateTimeFromRTC() {
  DateTime now = rtc.now();
  
  // Debug-Ausgabe
  Serial.print("RTC Zeit: ");
  Serial.print(now.hour()); Serial.print(":");
  Serial.print(now.minute()); Serial.print(":");
  Serial.println(now.second());
  
  hour = now.hour();
  minute = now.minute();
  second = now.second();
}

// Funktion zur Anzeige der Zeit auf dem Zeit-Display
void showTime() {
  displayTime.clearBuffer();
  
  DateTime now = rtc.now(); // Direkt vom RTC für Aktualität
  
  displayTime.setFont(u8g2_font_inb19_mf); // Größere Schrift für die Uhrzeit
  displayTime.setCursor(5, 30);
  
  // Formatieren der Zeit mit führenden Nullen
  char timeStr[9];
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  displayTime.print(timeStr);
  
  displayTime.setFont(u8g2_font_helvB10_tf); // Kleinere Schrift für das Datum
  displayTime.setCursor(15, 55);
  
  char dateStr[16]; // Größer für zusätzlichen UTC-Text
  sprintf(dateStr, "%02d.%02d.%04d UTC", now.day(), now.month(), now.year());
  displayTime.print(dateStr);
  
  displayTime.sendBuffer();
}

// Funktion zur Anzeige der RPM auf dem RPM-Display
void showRPM() {
  displayRPM.clearBuffer();
  
  displayRPM.setFont(u8g2_font_inb19_mf);
  displayRPM.setCursor(5, 30);
  displayRPM.print("RPM:");
  
  displayRPM.setFont(u8g2_font_inb24_mf); // Größte Schrift für den RPM-Wert
  displayRPM.setCursor(5, 60);
  displayRPM.print(Rpm);
  
  // Wenn keine SD-Karte verfügbar ist, "NO CARD" anzeigen
  if (!sdCardAvailable) {
    displayRPM.setFont(u8g2_font_helvB08_tf); // Kleinere Schrift für die Warnung
    displayRPM.setCursor(80, 20);
    displayRPM.print("NO");
    displayRPM.setCursor(80, 30);
    displayRPM.print("CARD");
  }
  
  displayRPM.sendBuffer();
}

// Alternative Initialisierungsmethode ohne direkte Kommandos
void initDisplayWithContrast() {
  // Explizite Adresszuweisung
  displayRPM.setI2CAddress(0x7A); // 8-bit
  
  // Einfachere Initialisierung
  displayRPM.begin();
  
  // Zeitverzögerung für Stabilisierung
  delay(200);
  
  // Display explizit einschalten
  displayRPM.setPowerSave(0);
  
  // Test-Anzeige
  displayRPM.clearBuffer();
  displayRPM.setFont(u8g2_font_inb24_mf);
  displayRPM.setCursor(15, 40);
  displayRPM.print("RPM INIT");
  displayRPM.sendBuffer();
  
  // Zusätzliche Verzögerung
  delay(100);
}

// Test beider Displays mit expliziten Adressen
void testDisplays() {
  // Explizite Adressenzuweisung für den RPM-Display-Test
  Wire.beginTransmission(0x3D); // 0x7A in 8-Bit = 0x3D in 7-Bit
  if (Wire.endTransmission() == 0) {
    Serial.println("RPM-Display auf 0x3D ist erreichbar");
    
    displayRPM.setI2CAddress(0x7A); // Direkte 8-Bit-Adresse
    displayRPM.clearBuffer();
    displayRPM.setFont(u8g2_font_inb24_mf);
    displayRPM.setCursor(15, 40);
    displayRPM.print("RPM TEST");
    displayRPM.sendBuffer();
    
    Serial.println("RPM-Display Test gesendet");
  } else {
    Serial.println("RPM-Display auf 0x3D ist NICHT erreichbar!");
  }
  delay(2000);
  
  // Explizite Adressenzuweisung für den Zeit-Display-Test
  Wire.beginTransmission(0x3C); // 0x78 in 8-Bit = 0x3C in 7-Bit
  if (Wire.endTransmission() == 0) {
    Serial.println("Time-Display auf 0x3C ist erreichbar");
    
    displayTime.setI2CAddress(0x78); // Direkte 8-Bit-Adresse
    displayTime.clearBuffer();
    displayTime.setFont(u8g2_font_inb24_mf);
    displayTime.setCursor(15, 40);
    displayTime.print("TIME TEST");
    displayTime.sendBuffer();
    
    Serial.println("Zeit-Display Test gesendet");
  } else {
    Serial.println("Zeit-Display auf 0x3C ist NICHT erreichbar!");
  }
  delay(2000);
}

// Funktion zur Anzeige der "NO CARD" Warnung auf dem Zeit-Display
void showNoCard() {
  displayTime.clearBuffer();
  
  // Deutliche, große Anzeige
  displayTime.setFont(u8g2_font_inb24_mf); // Größte verfügbare Schrift
  displayTime.setCursor(15, 30);
  displayTime.print("NO");
  displayTime.setCursor(15, 60);
  displayTime.print("CARD");
  
  displayTime.sendBuffer();
}

// Hilfsfunktion, um die maximale Tagesanzahl für einen Monat zu ermitteln
int daysInMonth(int month, int year) {
  switch (month) {
    case 2: // Februar
      // Schaltjahr-Prüfung: In einem Schaltjahr hat Februar 29 Tage
      if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
        return 29;
      } else {
        return 28;
      }
    case 4: // April
    case 6: // Juni
    case 9: // September
    case 11: // November
      return 30;
    default:
      return 31; // Für alle anderen Monate
  }
}

// Funktion zur Anzeige des Setup-Bildschirms auf beiden Displays
void displaySetup() {
  // Auf RPM-Display
  displayRPM.clearBuffer();
  displayRPM.setFont(u8g2_font_helvB12_tf);
  
  displayRPM.setCursor(5, 20);
  displayRPM.print("Settings");
  
  displayRPM.setCursor(5, 40);
  displayRPM.print("Mode");
  
  displayRPM.setCursor(5, 60);
  displayRPM.print("Use + / -");
  
  displayRPM.sendBuffer();
  
  // Auf Zeit-Display
  displayTime.clearBuffer();
  displayTime.setFont(u8g2_font_helvB12_tf);
  
  displayTime.setCursor(5, 20);
  
  if (currentState <= SET_YEAR) {
    displayTime.print("Setup Date");
  } else {
    displayTime.print("Setup Time");
  }
  
  displayTime.setCursor(5, 40);
  
  switch(currentState) {
    case SET_DAY:
      displayTime.print("Day: ");
      displayTime.print(day);
      break;
    case SET_MONTH:
      displayTime.print("Month: ");
      displayTime.print(month);
      break;
    case SET_YEAR:
      displayTime.print("Year: ");
      displayTime.print(year);
      break;
    case SET_HOUR:
      displayTime.print("Hour: ");
      displayTime.print(hour);
      break;
    case SET_MINUTE:
      displayTime.print("Minute: ");
      displayTime.print(minute);
      break;
    case SET_SECOND:
      displayTime.print("Second: ");
      displayTime.print(second);
      break;
    default:
      break;
  }
  
  displayTime.setCursor(5, 60);
  displayTime.print("+ / - to change");
  
  displayTime.sendBuffer();
}

void handleDayChange(bool increase) {
  if (increase) {
    day = (day % 31) + 1;
  } else {
    day = (day <= 1) ? 31 : day - 1;
  }
}

void handleMonthChange(bool increase) {
  if (increase) {
    month = (month % 12) + 1;
  } else {
    month = (month <= 1) ? 12 : month - 1;
  }
  // Tag korrigieren falls nötig
  int maxDays = daysInMonth(month, year);
  if (day > maxDays) {
    day = maxDays;
    stateChanged = true;
  }
}

// Funktion zum Einstellen des Datums und der Uhrzeit mit Tasten
void setupRTCWithButtons() {
  // Interrupt deaktivieren
  detachInterrupt(digitalPinToInterrupt(BUTTON_SET));
  
  // Aktuelle Zeit und Datum vom RTC holen für richtigen Startwert
  DateTime currentTime = rtc.now();
  day = currentTime.day();
  month = currentTime.month();
  year = currentTime.year();
  hour = currentTime.hour();
  minute = currentTime.minute();
  second = currentTime.second();
  
  // Mit Datumseinstellung beginnen
  currentState = SET_DAY;
  stateChanged = true;
  
  // Warte, bis die SET-Taste losgelassen wird
  while(digitalRead(BUTTON_SET) == LOW) {
    delay(10);
  }
  delay(200); // Extra Entprellung
  
  while (currentState != COMPLETE) {
    if (stateChanged) {
      displaySetup();
      stateChanged = false;
    }
    
    // PLUS-Taste abfragen
    if (digitalRead(BUTTON_PLUS) == LOW) {
      delay(200); // Entprellung
      
      switch(currentState) {
        case SET_DAY:
          handleDayChange(true);
          break;
        case SET_MONTH:
          handleMonthChange(true);
          break;
        case SET_YEAR:
          year = (year < 2050) ? year + 1 : 2020;
          // Korrektur für Februar in Nicht-Schaltjahren
          if (month == 2) {
            int maxDays = daysInMonth(month, year);
            if (day > maxDays) {
              day = maxDays;
              stateChanged = true;
            }
          }
          break;
        case SET_HOUR:
          hour = (hour + 1) % 24;
          break;
        case SET_MINUTE:
          minute = (minute + 1) % 60;
          break;
        case SET_SECOND:
          second = (second + 1) % 60;
          break;
        default:
          break;
      }
      
      stateChanged = true;
    }
    
    // MINUS-Taste abfragen
    if (digitalRead(BUTTON_MINUS) == LOW) {
      delay(200); // Entprellung
      
      switch(currentState) {
        case SET_DAY:
          handleDayChange(false);
          break;
        case SET_MONTH:
          handleMonthChange(false);
          break;
        case SET_YEAR:
          year = (year > 2020) ? year - 1 : 2050;
          // Korrektur für Februar in Nicht-Schaltjahren
          if (month == 2) {
            int maxDays = daysInMonth(month, year);
            if (day > maxDays) {
              day = maxDays;
              stateChanged = true;
            }
          }
          break;
        case SET_HOUR:
          hour = (hour + 23) % 24;
          break;
        case SET_MINUTE:
          minute = (minute + 59) % 60;
          break;
        case SET_SECOND:
          second = (second + 59) % 60;
          break;
        default:
          break;
      }
      
      stateChanged = true;
    }
    
    // SET-Taste abfragen
    if (digitalRead(BUTTON_SET) == LOW) {
      delay(200); // Entprellung
      
      switch(currentState) {
        case SET_DAY:
          currentState = SET_MONTH;
          break;
        case SET_MONTH:
          currentState = SET_YEAR;
          break;
        case SET_YEAR:
          currentState = SET_HOUR;
          break;
        case SET_HOUR:
          currentState = SET_MINUTE;
          break;
        case SET_MINUTE:
          currentState = SET_SECOND;
          break;
        case SET_SECOND:
          // Datum und Zeit im RTC speichern
          rtc.adjust(DateTime(year, month, day, hour, minute, second));
          currentState = COMPLETE;
          break;
        default:
          break;
      }
      
      stateChanged = true;
      
      // WICHTIG: Warte bis die SET-Taste losgelassen wird
      while(digitalRead(BUTTON_SET) == LOW) {
        delay(10);
      }
    }
  }
  
  // Bestätigungsnachricht anzeigen
  displayTime.clearBuffer();
  displayTime.setFont(u8g2_font_helvB12_tf);
  displayTime.setCursor(5, 30);
  displayTime.print("Date & Time Set!");
  displayTime.setCursor(5, 50);
  displayTime.print("Returning...");
  displayTime.sendBuffer();
  
  displayRPM.clearBuffer();
  displayRPM.setFont(u8g2_font_helvB12_tf);
  displayRPM.setCursor(5, 30);
  displayRPM.print("Settings");
  displayRPM.setCursor(5, 50);
  displayRPM.print("Complete!");
  displayRPM.sendBuffer();
  
  delay(1500);
  
  // Zurück zum Normalmodus
  currentState = NORMAL;
  // WICHTIG: Stelle sicher, dass alle Tasten losgelassen wurden
  while(digitalRead(BUTTON_SET) == LOW || 
        digitalRead(BUTTON_PLUS) == LOW || 
        digitalRead(BUTTON_MINUS) == LOW) {
    delay(10);
  }
  
  // Weitere Verzögerung nach dem Loslassen aller Tasten
  delay(200);
  
  // Interrupt erst jetzt wieder aktivieren
  attachInterrupt(digitalPinToInterrupt(BUTTON_SET), handleSetButtonInterrupt, FALLING);
}

// Funktion zum Generieren eines Dateinamens mit aktuellem Datum und Uhrzeit
String getNextFileName() {
  // Aktuelle Zeit vom RTC holen
  DateTime now = rtc.now();
  
  // Dateiname im Format: rpm_log_YYYY-MM-DD_HH-MM-SS.csv
  char fileName[50];
  sprintf(fileName, "/rpm_log_%04d-%02d-%02d_%02d-%02d-%02d.csv", 
          now.year(), now.month(), now.day(), 
          now.hour(), now.minute(), now.second());
  
  // In String umwandeln für einfacheres Handling
  String fileNameStr = String(fileName);
  
  Serial.print("Neuer Dateiname mit Zeitstempel: ");
  Serial.println(fileNameStr);
  
  return fileNameStr;
}

// Funktion zur Überprüfung der SD-Karte während der Laufzeit
bool checkSDCard() {
  // Zuerst prüfen, ob die Karte grundsätzlich initialisiert ist
  if (!SD.begin(SD_CS)) {
    return false;
  }
  
  // Zusätzlich testen, ob wir lesen/schreiben können
  if (!SD.exists("/rpm_log.csv")) {
    // Versuchen, die Datei zu erstellen, wenn sie nicht existiert
    File testFile = SD.open("/rpm_log.csv", FILE_WRITE);
    if (testFile) {
      // Datei konnte geöffnet werden
      testFile.close();
      return true;
    }
    return false;
  }
  
  return true;
}

// Im setup() nach Initialisierung des IR-Sensors
void testIRSensor() {
  displayRPM.clearBuffer();
  displayRPM.setFont(u8g2_font_helvB12_tf);
  displayRPM.setCursor(0, 20);
  displayRPM.print("Testing IR...");
  displayRPM.setCursor(0, 40);
  displayRPM.print("Move object");
  displayRPM.sendBuffer();
  
  // Lokale Zähler für den Test
  int signalCount = 0;
  unsigned long testStart = millis();
  
  // Registriere für 5 Sekunden alle Signale
  while (millis() - testStart < 5000) {
    int sensorState = digitalRead(IR_SENSOR_PIN);
    if (sensorState == LOW) { // Angepasst an deine Sensorlogik
      signalCount++;
      digitalWrite(LED_PIN, HIGH); // Visuelle Rückmeldung
    } else {
      digitalWrite(LED_PIN, LOW);
    }
    delay(10); // Polling-Häufigkeit
  }
  
  // Ergebnis anzeigen
  displayRPM.clearBuffer();
  displayRPM.setCursor(0, 20);
  displayRPM.print("IR Test Result:");
  displayRPM.setCursor(0, 40);
  displayRPM.print("Signals: ");
  displayRPM.print(signalCount);
  displayRPM.sendBuffer();
  
  delay(2000);
}

// Haupteinrichtung
void setup() {
  Serial.begin(115200);
  Serial.println("Setup gestartet");
  
  // I2C-Bus initialisieren
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // I2C-Scanner durchführen
  scanI2C();
  delay(500);
  
  // Pin-Modi festlegen
  pinMode(BUTTON_PLUS, INPUT_PULLUP);
  pinMode(BUTTON_MINUS, INPUT_PULLUP);
  pinMode(BUTTON_SET, INPUT_PULLUP);
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Im setup()
  // Displays initialisieren
  if (!displayRPM.begin()) {
    Serial.println("RPM Display (0x7A) konnte nicht initialisiert werden");
    // Alternativer Versuch mit expliziter Adresse
    displayRPM.setI2CAddress(0x7A);
    if (!displayRPM.begin()) {
      Serial.println("Zweiter Versuch fehlgeschlagen");
    }
  }else {
    Serial.println("RTC Display erfolgreich initialisiert");
  }
  
  if (!displayTime.begin()) {
    Serial.println("Time Display (0x78) konnte nicht initialisiert werden");
  } else {
    Serial.println("Time Display erfolgreich initialisiert");
  }

  // Speziell für das RPM-Display
  initDisplayWithContrast();
  testDisplays();
  // Willkommensnachricht auf beiden Displays anzeigen
  displayRPM.clearBuffer();
  displayRPM.setFont(u8g2_font_helvB12_tf);
  displayRPM.setCursor(0, 20);
  displayRPM.print("RPM Sensor");
  displayRPM.setCursor(0, 40);
  displayRPM.print("RPM Display");
  displayRPM.sendBuffer();
  
  displayTime.clearBuffer();
  displayTime.setFont(u8g2_font_helvB12_tf);
  displayTime.setCursor(0, 20);
  displayTime.print("RPM Sensor");
  displayTime.setCursor(0, 40);
  displayTime.print("Time Display");
  displayTime.sendBuffer();
  
  delay(1000);
  
  // RTC initialisieren
  displayTime.clearBuffer();
  displayTime.setCursor(0, 20);
  displayTime.print("Initializing");
  displayTime.setCursor(0, 40);
  displayTime.print("RTC...");
  displayTime.sendBuffer();
  
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC!");
    displayTime.clearBuffer();
    displayTime.setCursor(0, 30);
    displayTime.print("RTC Error!");
    displayTime.sendBuffer();
    
    while (1) delay(10);
  }
  
  rtc.disable32K();
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);
  rtc.disableAlarm(2);
  
  // RTC Status anzeigen
  displayTime.clearBuffer();
  displayTime.setCursor(0, 20);
  displayTime.print("RTC OK");
  displayTime.sendBuffer();
  
  // Erste Zeitübernahme vom RTC mit Debug-Ausgabe
  DateTime initialTime = rtc.now();
  Serial.print("Initial RTC time: ");
  Serial.print(initialTime.year()); Serial.print("-");
  Serial.print(initialTime.month()); Serial.print("-");
  Serial.print(initialTime.day()); Serial.print(" ");
  Serial.print(initialTime.hour()); Serial.print(":");
  Serial.print(initialTime.minute()); Serial.print(":");
  Serial.println(initialTime.second());
  
  // Zeit aus RTC holen oder bei Bedarf einstellen
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting default time!");
    displayTime.clearBuffer();
    displayTime.setCursor(0, 20);
    displayTime.print("RTC lost power");
    displayTime.setCursor(0, 40);
    displayTime.print("Setting time...");
    displayTime.sendBuffer();
    
    // Standardzeit setzen (22.04.2025 12:00:00)
    rtc.adjust(DateTime(2025, 4, 22, 12, 0, 0));
    delay(1000);
  } else {
    displayTime.clearBuffer();
    displayTime.setCursor(0, 20);
    displayTime.print("RTC time:");
    displayTime.setCursor(0, 40);
    
    DateTime now = rtc.now();
    char timeStr[20];
    sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    displayTime.print(timeStr);
    displayTime.sendBuffer();
    delay(1000);
  }
  
  updateTimeFromRTC();
  
  // SD-Karte initialisieren
  displayRPM.clearBuffer();
  displayRPM.setCursor(0, 20);
  displayRPM.print("Initializing");
  displayRPM.setCursor(0, 40);
  displayRPM.print("SD card...");
  displayRPM.sendBuffer();
  
  sdCardAvailable = checkSDCard();
  if (sdCardAvailable) {
    Serial.println("SD-Karte erfolgreich initialisiert");
    
    displayRPM.clearBuffer();
    displayRPM.setCursor(0, 20);
    displayRPM.print("SD Card OK");
    displayRPM.sendBuffer();
    
    // Neuen Dateinamen generieren
    currentLogFileName = getNextFileName();
    Serial.print("Neuer Log-Dateiname: ");
    Serial.println(currentLogFileName);
    
    // Log-Header schreiben
    File dataFile = SD.open(currentLogFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("Date,UTC,RPM,Temperatur");
      dataFile.close();
      
      // Bei der Anzeige des Dateinamens
      displayRPM.clearBuffer();
      displayRPM.setFont(u8g2_font_helvB10_tf);  // Etwas kleinere Schrift für mehr Platz
      displayRPM.setCursor(0, 15);
      displayRPM.print("Log file:");
      
      // Zeigt nur Datum und Uhrzeit an, ohne Pfad und Präfix
      String shortFileName = currentLogFileName;
      shortFileName.replace("/rpm_log_", "");  // Entferne Pfad und Präfix
      shortFileName.replace(".csv", "");       // Entferne Dateiendung
      
      // Aufteilen in zwei Zeilen
      int underscorePos = shortFileName.indexOf('_');
      if (underscorePos != -1) {
        // Datum
        displayRPM.setCursor(0, 35);
        displayRPM.print(shortFileName.substring(0, underscorePos));
        
        // Zeit
        displayRPM.setCursor(0, 55);
        displayRPM.print(shortFileName.substring(underscorePos + 1));
      } else {
        // Falls kein Unterstrich vorhanden ist, zeige den gesamten Namen
        displayRPM.setCursor(0, 35);
        displayRPM.print(shortFileName);
      }
      
      displayRPM.sendBuffer();
    }
  } else {
    Serial.println("SD-Karte konnte nicht initialisiert werden!");
    
    displayRPM.clearBuffer();
    displayRPM.setCursor(0, 20);
    displayRPM.print("SD Card Error!");
    displayRPM.setCursor(0, 40);
    displayRPM.print("Logging disabled");
    displayRPM.sendBuffer();
    
    // Zeige NO CARD Warnung auf dem Zeit-Display
    showNoCard();
  }
  
  delay(1000);
  
  // RPM Sensor initialisieren
  displayRPM.clearBuffer();
  displayRPM.setCursor(0, 20);
  displayRPM.print("Initializing");
  displayRPM.setCursor(0, 40);
  displayRPM.print("RPM sensor...");
  displayRPM.sendBuffer();

  testIRSensor(); // Teste den IR-Sensor und zeige die Ergebnisse an
  delay(10000);
  Rpm_Count = 0;
  lastTime = millis();
  lastSecondRpmCount = millis();
  Rpm_Count_LastSecond = 0;
  lastOutputTime = millis();
    // attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), Rpm_isr, CHANGE);
  // oder
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), Rpm_isr, FALLING);
  
  // SET-Taste Interrupt konfigurieren
  attachInterrupt(digitalPinToInterrupt(BUTTON_SET), handleSetButtonInterrupt, FALLING);
  
  // Setup abgeschlossen
  displayRPM.clearBuffer();
  displayRPM.setCursor(0, 20);
  displayRPM.print("Setup");
  displayRPM.setCursor(0, 40);
  displayRPM.print("Complete!");
  displayRPM.sendBuffer();
  
  delay(1500);
  
  // Initialisiere die Anzeigen
  updateTimeFromRTC();
  showTime(); // Korrigiert von displayTime()
  showRPM(); // Korrigiert von displayRPM()
  
  Serial.println("Setup abgeschlossen");
}

void loop() {
  unsigned long currentTime = millis();
  static bool handlingButton = false;
  static int lastRpm = -1;
  static unsigned long lastCardCheck = 0;
  
  // SD-Karten-Status regelmäßig überprüfen (alle 3 Sekunden)
  if (currentTime - lastCardCheck >= 3000) {
    bool previousState = sdCardAvailable;
    sdCardAvailable = checkSDCard();
    
    // Bei Statusänderung entsprechende Aktion durchführen
    if (previousState != sdCardAvailable) {
      if (sdCardAvailable) {
        Serial.println("SD-Karte angeschlossen");
        
        // Neuen Dateinamen generieren
        currentLogFileName = getNextFileName();
        Serial.print("Neuer Log-Dateiname: ");
        Serial.println(currentLogFileName);
        
        // Neuen Log-Header schreiben
        File dataFile = SD.open(currentLogFileName, FILE_WRITE);
        if (dataFile) {
          dataFile.println("Datum,Zeit,RPM,Temperatur");
          dataFile.close();
          
          // Kurze Bestätigung anzeigen
          displayRPM.clearBuffer();
          displayRPM.setFont(u8g2_font_helvB12_tf);
          displayRPM.setCursor(0, 20);
          displayRPM.print("New log file:");
          displayRPM.setCursor(0, 40);
          String shortFileName = currentLogFileName.substring(1);
          displayRPM.print(shortFileName);
          displayRPM.sendBuffer();
          delay(2000);
          
          // Normale Zeit wieder anzeigen auf Zeit-Display
          showTime(); // Korrigiert von displayTime()
          // Normale RPM wieder anzeigen
          showRPM(); // Korrigiert von displayRPM()
        }
      } else {
        Serial.println("SD-Karte entfernt");
        // Auf Zeit-Display NO CARD-Warnung anzeigen
        showNoCard(); // Korrigiert von displayNoCard()
        // RPM-Display aktualisieren um den NO CARD-Indikator zu zeigen
        showRPM(); // Korrigiert von displayRPM()
      }
    }
    
    lastCardCheck = currentTime;
  }
  
  // RPM berechnen (einmal pro Sekunde)
   // Verbesserte RPM-Berechnung in der loop()-Funktion
  if (currentTime - lastSecondRpmCount >= 1000) {
    // Bei sehr langsamen Drehzahlen kann es sein, dass innerhalb von 1 Sekunde
    // gar kein Impuls kommt, deshalb den Maximalwert von 0 setzen
    if (Rpm_Count_LastSecond == 0) {
      Rpm = 0;
    } else {
      Rpm = Rpm_Count_LastSecond * 60 / RpmTriggerPerRound;
    }
    
    // Debug-Ausgabe für Störungssuche
    Serial.print("Raw count: ");
    Serial.print(Rpm_Count_LastSecond);
    Serial.print(", Calculated RPM: ");
    Serial.println(Rpm);
    
    Rpm_Count_LastSecond = Rpm_Count;
    Rpm_Count = 0;
    lastSecondRpmCount = currentTime;
  
    // Temperatur vom RTC-Modul auslesen
    float temperature = rtc.getTemperature();
    
    // Logge die Daten auf die SD-Karte nur wenn sie verfügbar ist
    if (sdCardAvailable) {
      File dataFile = SD.open(currentLogFileName, FILE_APPEND);
      if (dataFile) {
        DateTime now = rtc.now();
        
        // Datums- und Zeitformat: YYYY-MM-DD,HH:MM:SS
        dataFile.print(now.year()); dataFile.print("-");
        dataFile.print(now.month()); dataFile.print("-");
        dataFile.print(now.day()); dataFile.print(",");
        
        dataFile.print(now.hour()); dataFile.print(":");
        dataFile.print(now.minute()); dataFile.print(":");
        dataFile.print(now.second()); dataFile.print(",");
        
        dataFile.print(Rpm); dataFile.print(",");
        dataFile.println(temperature);
        
        dataFile.close();
      } else {
        sdCardAvailable = false; // Fehler beim Schreiben, Status aktualisieren
      }
    }
    
    // RPM-Anzeige aktualisieren
    showRPM(); // Korrigiert von displayRPM()
  }
  
  // Zeit-Anzeige jede Sekunde aktualisieren
  if (currentTime - lastOutputTime >= 1000) {
    if (sdCardAvailable) {
      updateTimeFromRTC();
      showTime(); // Korrigiert von displayTime()
    } else {
      // Bei fehlender SD-Karte "NO CARD" anzeigen
      showNoCard(); // Korrigiert von displayNoCard()
    }
    lastOutputTime = currentTime;
  }
  
  // Überprüfen, ob die SET-Taste gedrückt wurde
  if (buttonPressed && !handlingButton) {
    handlingButton = true; // Flag setzen
    buttonPressed = false;
    delay(200); // Entprellung
    setupRTCWithButtons();
    
    // Nach dem Setup Displays neu zeichnen
    updateTimeFromRTC();
    showTime(); // Korrigiert von displayTime()
    showRPM(); // Korrigiert von displayRPM()
    
    delay(1000); // Längere Verzögerung nach dem Setup
    
    // Stelle sicher, dass keine Taste gedrückt ist
    while(digitalRead(BUTTON_SET) == LOW) {
      delay(10);
    }
    
    // Reset buttonPressed explizit nochmal
    buttonPressed = false;
    handlingButton = false; // Flag zurücksetzen
  }
}
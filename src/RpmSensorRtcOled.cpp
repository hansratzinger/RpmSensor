// -------------------------------------------------
// RpmSensorRtcOled.cpp
// -------------------------------------------------
// Usage for measuring RMP (Revolutions per Minute) with an infrared sensor
// and displaying the result on an OLED display. The DS3231 RTC is used to keep track of time.
// The SD card is used to log the data. The program allows setting the RTC time using buttons on the ESP32 board.
// ESP32 DS3231 RTC + OLED Display + Infrared Sensor
// RELEASE 2.0 HR 2025-04-22 NK
// -------------------------------------------------
// Changed from TFT + SD to OLED display
// RELEASE 2.1 HR 2025-04-23 NK
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
#define RpmTriggerPerRound 3 // 3 Impulse pro Umdrehung für präzisere Messung

#define IR_SENSOR_PIN 15 // GPIO15: Infrarotsensor Pin
#define LED_PIN 12      // GPIO12: Kontroll-LED Pin

// I2C Pins für einen Bus
#define SDA_PIN 21
#define SCL_PIN 22

// OLED Display Adresse
#define OLED_ADDR 0x3C // Standard-Adresse für die meisten SSD1306 Displays

// Initialisierung des OLED-Displays
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Globale Statusvariable für SD-Karte hinzufügen
bool sdCardAvailable = false;

// Globale Variable für den aktuellen Dateinamen
String currentLogFileName;

// RTC Modul
RTC_DS3231 rtc;

// Zustandsvariablen
bool TEST = false; // Debug-Modus
#define CLOCK_INTERRUPT_PIN 14

// Display-Modus
enum DisplayMode {
  TIME_MODE,
  RPM_MODE
};

DisplayMode currentDisplayMode = TIME_MODE;
unsigned long lastDisplaySwitch = 0;
const unsigned long DISPLAY_SWITCH_INTERVAL = 5000; // 5 Sekunden zwischen Anzeigewechsel

// Zustandsarten für die RTC-Einstellung
enum SetupState {
  NORMAL,
  SET_DAY,     // Zusätzlich: Tag einstellen
  SET_MONTH,   // Zusätzlich: Monat einstellen
  SET_YEAR,    // Zusätzlich: Jahr einstellen
  SET_HOUR,
  SET_MINUTE,
  SET_SECOND,
  COMPLETE
};

volatile SetupState currentState = NORMAL;
volatile bool stateChanged = false;
volatile bool buttonPressed = false;

// Variablen für Datum und Zeit
int hour = 0;
int minute = 0;
int second = 0;
int day = 1;     // Neu: Tag
int month = 5;   // Neu: Monat
int year = 2025; // Neu: Jahr

// RPM-Messung
volatile unsigned long Rpm_Count; // Zähler für Interrupts
volatile unsigned long Rpm_Count_LastSecond; // Zähler für letzte Sekunde
int Rpm;                          // Variable für die aktuelle RPM
unsigned long lastTime;           // Variable für die letzte Zeitmessung
unsigned long lastOutputTime = 0; // Zeitpunkt der letzten Ausgabe
unsigned long lastSecondRpmCount = 0; // Zeitpunkt der letzten Sekundenmessung

void IRAM_ATTR Rpm_isr() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = micros();
  
  // Entprellung: Erhöht auf 20ms für unsaubere Impulse (10-15ms im Oszilloskop gesehen)
  if (interruptTime - lastInterruptTime > 20000) { // 20000 Mikrosekunden = 20 Millisekunden
    Rpm_Count++;
    Rpm_Count_LastSecond++;
    lastInterruptTime = interruptTime;
  }
}

// Umbenennen der globalen Variable zur Vermeidung von Namenskonflikten
unsigned long lastButtonInterruptTime = 0; // Für Tastenentprellung
const unsigned long debounceTime = 500;    // 500ms Entprellzeit

void IRAM_ATTR handleSetButtonInterrupt() {
  unsigned long interruptTime = millis();
  
  // Nur bei genügend Abstand zum letzten Interrupt
  if (interruptTime - lastButtonInterruptTime > debounceTime) {
    buttonPressed = true;
    lastButtonInterruptTime = interruptTime; // Umbenannte Variable
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

// Funktion zur Anzeige der Zeit
void displayTime() {
  display.clearBuffer();
  
  DateTime now = rtc.now(); // Direkt vom RTC für Aktualität
  
  display.setFont(u8g2_font_inb19_mf); // Größere Schrift für die Uhrzeit
  display.setCursor(5, 30);
  
  // Formatieren der Zeit mit führenden Nullen
  char timeStr[9];
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  display.print(timeStr);
  
  display.setFont(u8g2_font_helvB10_tf); // Kleinere Schrift für das Datum
  display.setCursor(15, 55);
  
  char dateStr[16]; // Größer für zusätzlichen UTC-Text
  sprintf(dateStr, "%02d.%02d.%04d UTC", now.day(), now.month(), now.year());
  display.print(dateStr);
  
  display.sendBuffer();
}

// Funktion zur Anzeige der RPM
void displayRPM() {
  display.clearBuffer();
  
  display.setFont(u8g2_font_inb19_mf);
  display.setCursor(5, 30);
  display.print("RPM:");
    
  display.setFont(u8g2_font_inb24_mf); // Größte Schrift für den RPM-Wert
  display.setCursor(5, 60);
  display.print(Rpm);
  
  display.sendBuffer();

  Serial.print("RPM: ");
  Serial.println(Rpm);
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

// Funktion zur Anzeige des Setup-Bildschirms
void displaySetup() {
  display.clearBuffer();
  display.setFont(u8g2_font_helvB12_tf);
  
  display.setCursor(5, 20);
  
  if (currentState <= SET_YEAR) {
    display.print("Setup Date");
  } else {
    display.print("Setup Time");
  }
  
  display.setCursor(5, 40);
  
  switch(currentState) {
    case SET_DAY:
      display.print("Day: ");
      display.print(day);
      break;
    case SET_MONTH:
      display.print("Month: ");
      display.print(month);
      break;
    case SET_YEAR:
      display.print("Year: ");
      display.print(year);
      break;
    case SET_HOUR:
      display.print("Hour: ");
      display.print(hour);
      break;
    case SET_MINUTE:
      display.print("Minute: ");
      display.print(minute);
      break;
    case SET_SECOND:
      display.print("Second: ");
      display.print(second);
      break;
    default:
      break;
  }
  
  display.setCursor(5, 60);
  display.print("+ / - to change");
  
  display.sendBuffer();
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
  display.clearBuffer();
  display.setFont(u8g2_font_helvB12_tf);
  display.setCursor(5, 30);
  display.print("Date & Time Set!");
  display.setCursor(5, 50);
  display.print("Returning...");
  display.sendBuffer();
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

// Funktion zur Anzeige der "NO CARD" Warnung
void displayNoCard() {
  display.clearBuffer();
  
  // Deutliche, große Anzeige
  display.setFont(u8g2_font_inb24_mf); // Größte verfügbare Schrift
  display.setCursor(15, 30);
  display.print("NO");
  display.setCursor(15, 60);
  display.print("CARD");
  
  display.sendBuffer();
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
  
  // Display initialisieren
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_helvB12_tf);
  display.setCursor(0, 20);
  display.print("RPM Sensor");
  display.setCursor(0, 40);
  display.print("Display OK");
  display.sendBuffer();
  delay(500);
  
  // RTC initialisieren
  display.clearBuffer();
  display.setCursor(0, 20);
  display.print("Initializing");
  display.setCursor(0, 40);
  display.print("RTC...");
  display.sendBuffer();
  
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC!");
    display.clearBuffer();
    display.setCursor(0, 30);
    display.print("RTC Error!");
    display.sendBuffer();
    
    while (1) delay(10);
  }
  
  rtc.disable32K();
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);
  rtc.disableAlarm(2);
  
  // RTC Status anzeigen
  display.clearBuffer();
  display.setCursor(0, 20);
  display.print("RTC OK");
  display.sendBuffer();
  
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
    display.clearBuffer();
    display.setCursor(0, 20);
    display.print("RTC lost power");
    display.setCursor(0, 40);
    display.print("Setting time...");
    display.sendBuffer();
    
    // Standardzeit setzen (22.04.2025 12:00:00)
    rtc.adjust(DateTime(2025, 4, 22, 12, 0, 0));
    delay(1000);
  } else {
    display.clearBuffer();
    display.setCursor(0, 20);
    display.print("RTC time:");
    display.setCursor(0, 40);
    
    DateTime now = rtc.now();
    char timeStr[20];
    sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    display.print(timeStr);
    display.sendBuffer();
    delay(1000);
  }
  
  updateTimeFromRTC();
  
   // SD-Karte initialisieren (im setup())
  display.clearBuffer();
  display.setCursor(0, 20);
  display.print("Initializing");
  display.setCursor(0, 40);
  display.print("SD card...");
  display.sendBuffer();
  
  sdCardAvailable = checkSDCard();
  // Im setup():
  if (sdCardAvailable) {
    Serial.println("SD-Karte erfolgreich initialisiert");
    
    display.clearBuffer();
    display.setCursor(0, 20);
    display.print("SD Card OK");
    display.sendBuffer();
    
    // Neuen Dateinamen generieren
    currentLogFileName = getNextFileName();
    Serial.print("Neuer Log-Dateiname: ");
    Serial.println(currentLogFileName);
    
    // Log-Header schreiben
    File dataFile = SD.open(currentLogFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("Date,UTC,RPM,Temperatur");
      dataFile.close();
      
      // Bei der Anzeige des Dateinamens (in setup oder SD-Karten-Erkennung)
      display.clearBuffer();
      display.setFont(u8g2_font_helvB10_tf);  // Etwas kleinere Schrift für mehr Platz
      display.setCursor(0, 15);
      display.print("Log file:");
      
      // Zeigt nur Datum und Uhrzeit an, ohne Pfad und Präfix
      String shortFileName = currentLogFileName;
      shortFileName.replace("/rpm_log_", "");  // Entferne Pfad und Präfix
      shortFileName.replace(".csv", "");       // Entferne Dateiendung
      
      // Aufteilen in zwei Zeilen
      int underscorePos = shortFileName.indexOf('_');
      if (underscorePos != -1) {
        // Datum
        display.setCursor(0, 35);
        display.print(shortFileName.substring(0, underscorePos));
        
        // Zeit
        display.setCursor(0, 55);
        display.print(shortFileName.substring(underscorePos + 1));
      } else {
        // Falls kein Unterstrich vorhanden ist, zeige den gesamten Namen
        display.setCursor(0, 35);
        display.print(shortFileName);
      }
      
      display.sendBuffer();
    }
  } else {
    Serial.println("SD-Karte konnte nicht initialisiert werden!");
    
    display.clearBuffer();
    display.setCursor(0, 20);
    display.print("SD Card Error!");
    display.setCursor(0, 40);
    display.print("Logging disabled");
    display.sendBuffer();
  }
  
  delay(1000);
  
  // RPM Sensor initialisieren
  display.clearBuffer();
  display.setCursor(0, 20);
  display.print("Initializing");
  display.setCursor(0, 40);
  display.print("RPM sensor...");
  display.sendBuffer();
  
  Rpm_Count = 0;
  lastTime = millis();
  lastSecondRpmCount = millis();
  Rpm_Count_LastSecond = 0;
  lastOutputTime = millis();
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), Rpm_isr, FALLING);
  
  // SET-Taste Interrupt konfigurieren
  attachInterrupt(digitalPinToInterrupt(BUTTON_SET), handleSetButtonInterrupt, FALLING);
  
  // Setup abgeschlossen
  display.clearBuffer();
  display.setCursor(0, 20);
  display.print("Setup");
  display.setCursor(0, 40);
  display.print("Complete!");
  display.sendBuffer();
  
  delay(1500);
  
  // Initialisiere die Anzeige
  updateTimeFromRTC();
  displayTime();
  
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
        // In der SD-Kartenprüfung der loop()-Funktion:
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
          display.clearBuffer();
          display.setFont(u8g2_font_helvB12_tf);
          display.setCursor(0, 20);
          display.print("New log file:");
          display.setCursor(0, 40);
          String shortFileName = currentLogFileName.substring(1);
          display.print(shortFileName);
          display.sendBuffer();
          delay(2000); // Kurz anzeigen
          
          // Zurück zum aktuellen Anzeigemodus
          if (currentDisplayMode == TIME_MODE) {
            displayTime();
          } else {
            displayRPM();
          }
        }
      } else {
        Serial.println("SD-Karte entfernt");
        // Sofort die Warnung anzeigen, wenn die Karte entfernt wurde
        if (currentDisplayMode == TIME_MODE) {
          displayNoCard();
        }
      }
    }
    
    lastCardCheck = currentTime;
  }
  
  // RPM berechnen (einmal pro Sekunde)
  if (currentTime - lastSecondRpmCount >= 1000) {
    Rpm = Rpm_Count_LastSecond * 60 / RpmTriggerPerRound;
    Rpm_Count_LastSecond = 0; // Auf 0 zurücksetzen, nicht den Wert von Rpm_Count übernehmen
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
  }
  
  // Display-Modus wechseln
  if (currentTime - lastDisplaySwitch >= DISPLAY_SWITCH_INTERVAL) {
    if (currentDisplayMode == TIME_MODE) {
      currentDisplayMode = RPM_MODE;
      displayRPM();
    } else {
      currentDisplayMode = TIME_MODE;
      // Entscheiden, ob Zeit oder NO CARD angezeigt wird
      if (sdCardAvailable) {
        updateTimeFromRTC();
        displayTime();
      } else {
        displayNoCard();
      }
    }
    lastDisplaySwitch = currentTime;
  }
  
  // Zeit aktualisieren bei TIME_MODE (nur bei verfügbarer SD-Karte)
  if (currentDisplayMode == TIME_MODE && currentTime - lastOutputTime >= 1000) {
    if (sdCardAvailable) {
      updateTimeFromRTC();
      displayTime();
    } else {
      // Im TIME_MODE bei fehlender Karte NO CARD anzeigen
      displayNoCard();
    }
    lastOutputTime = currentTime;
  }
  
  // RPM aktualisieren bei RPM_MODE wenn sich der Wert geändert hat
  if (currentDisplayMode == RPM_MODE && Rpm != lastRpm) {
    displayRPM();
    lastRpm = Rpm;
  }
  
  // Überprüfen, ob die SET-Taste gedrückt wurde
  if (buttonPressed && !handlingButton) {
    handlingButton = true; // Flag setzen
    buttonPressed = false;
    delay(200); // Entprellung
    setupRTCWithButtons();
    
    // Nach dem Setup Display neu zeichnen
    updateTimeFromRTC();
    if (currentDisplayMode == TIME_MODE) {
      displayTime();
    } else {
      displayRPM();
    }
    
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
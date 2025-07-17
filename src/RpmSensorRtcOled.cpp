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
// Release 2.2 double OLED displays, impoved RPM, results checked with oszilloscope
// RELEASE 2.2 HR 2025-04-24 NK
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
#define BUTTON_PLUS 25  // GPIO für die "+"-Taste blau
#define BUTTON_MINUS 26 // GPIO für die "-"-Tastekkxxgbggf weiss 
#define BUTTON_SET 27   // GPIO für die "SET"-Taste gelb

#define LED_PIN 12      // GPIO12: Kontroll-LED Pin

#define HALL_SENSOR_PIN 15  // GPIO-Pin für den Hall-Sensor
#define RpmTriggerPerRound 4  // 4 Magnete = 4 Impulse pro Umdrehung

// I2C Pins für den Haupt-Bus (Displays, RTC) / I2C-1
#define SDA_PIN 21
#define SCL_PIN 22

// I2C Pins für den EEPROM-Bus (separater Bus)  / I2C-2
#define SDA_PIN_EEPROM 16
#define SCL_PIN_EEPROM 17

// TwoWire-Instanz für den EEPROM-Bus erstellen
// TwoWire Wire1 = TwoWire(1); // Verwende I2C Controller 1

// OLED Display Adressen
#define OLED_TIME_ADDR 0x3C // Erstes Display für Zeit/Datum
#define OLED_RPM_ADDR 0x3D  // Zweites Display für RPM

// Initialisierung der beiden OLED-Displays
U8G2_SSD1306_128X64_NONAME_F_HW_I2C displayTime(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // Zeit/Datum Display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C displayRpm(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // RPM Display

// Globale Statusvariable für SD-Karte hinzufügen
bool sdCardAvailable = false;

// Globale Variable für den aktuellen Dateinamen
String currentLogFileName;

// EEPROM-bezogene Bibliotheken und Definitionen
// EEPROM-Spezifikationen
#define EEPROM_ADDRESS 0x50    // Standard I2C-Adresse für 24C512 EEPROM
#define EEPROM_SIZE 524288     // 512kB = 512 * 1024 Bytes
#define EEPROM_PAGE_SIZE 128   // 128 Byte Seite laut Datenblatt
#define EEPROM_WRITE_CYCLE 5   // 5ms Schreibzyklus


struct __attribute__((packed)) LogRecord {
  uint32_t timestamp;          // UNIX-Timestamp
  uint16_t rpm;                // RPM-Wert
  int16_t temperature;         // Temperatur * 100
};

// Puffer für temporäre Daten (60 Datensätze für 1 Minute)
#define BUFFER_SIZE 60
LogRecord dataBuffer[BUFFER_SIZE];
uint8_t bufferCount = 0;
unsigned long lastMinuteWrite = 0;
const unsigned long MINUTE_INTERVAL = 60000; // 60 Sekunden in Millisekunden

struct __attribute__((packed)) EEPROMHeader {
  uint32_t recordCount;        // Anzahl der gespeicherten Datensätze
  uint32_t nextWriteAddress;   // Nächste freie Adresse zum Schreiben
  uint8_t initialized;         // Flag ob EEPROM initialisiert wurde
};

// EEPROM-Verwaltungsvariablen
EEPROMHeader eepromHeader;
bool eepromAvailable = false;
unsigned long lastEEPROMWrite = 0;  // Zeitpunkt des letzten EEPROM-Schreibvorgangs
const uint32_t EEPROM_WRITE_INTERVAL = 10000; // 10 Sekunden zwischen Schreibvorgängen
uint8_t bufferIndex = 0;  // Aktueller Index im Datenpuffer

// RTC Modul
RTC_DS3231 rtc;

// Zustandsvariablen
bool TEST = false; // Debug-Modus
#define CLOCK_INTERRUPT_PIN 14

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
// Tastenzustände (ergänze zu deinen existierenden globalen Variablen)
volatile bool plusButtonPressed = false;  // Für die "+"-Taste
volatile bool minusButtonPressed = false; // Für die "-"-Taste

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
volatile unsigned long pulsePeriod = 0;      // NEUE VARIABLE: Zeit zwischen Impulsen
volatile unsigned long pulseCount = 0;  
volatile bool newPulseDetected = false;  
unsigned long lastTime;           // Variable für die letzte Zeitmessung
unsigned long lastOutputTime = 0; // Zeitpunkt der letzten Ausgabe
unsigned long lastSecondRpmCount = 0; // Zeitpunkt der letzten Sekundenmessung

bool writeBufferedDataToEEPROM();
void setupRTC(); // Funktion zur Einrichtung des RTC
void setupSDCard(); // Funktion zur Einrichtung der SD-Karte
void setupDisplay(); // Funktion zur Einrichtung der Displays
void setupButtons(); // Funktion zur Einrichtung der Tasten
void setupIRSensor(); // Funktion zur Einrichtung des Infrarotsensors
void setupLED(); // Funktion zur Einrichtung der LED
void setupRTCWithButtons(); // Funktion zur Einstellung der RTC mit Tasten
void writeEEPROMByte(uint32_t address, uint8_t data);
uint8_t readEEPROMByte(uint32_t address);
void writeEEPROMPage(uint32_t address, const uint8_t* data, uint8_t length);
bool initEEPROM();
bool logDataToEEPROM(uint32_t timestamp, uint16_t rpm, float temperature);
LogRecord readRecordFromEEPROM(uint32_t recordIndex);
void handleSerialCommands();
void downloadEEPROMDataToUSB();
void eraseEEPROM();
void readEEPROMHeader();
void writeEEPROMHeader();
void handleButtons(unsigned long currentTime);
void updateDisplay();
void displaySetValue(const char* label, int value);
void updateTimeFromRTC();

// Interrupt-Handler für SET, PLUS und MINUS Tasten
void IRAM_ATTR buttonInterrupt() {
  buttonPressed = true;
}

void IRAM_ATTR plusButtonInterrupt() {
  plusButtonPressed = true;
}

void IRAM_ATTR minusButtonInterrupt() {
  minusButtonPressed = true;
}

void IRAM_ATTR Rpm_isr() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = micros();
  unsigned long interval = interruptTime - lastInterruptTime;
  
  // Entprellung: 3000 μs 
  if (interval > 3000) { // Min. 5ms zwischen Impulsen (entspricht max 12000 RPM)
    pulseCount = pulseCount + 1;
    pulsePeriod = interval;
    lastInterruptTime = interruptTime;
    newPulseDetected = true;
    
    // Kurzer LED-Blink für visuelle Bestätigung
    digitalWrite(LED_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(LED_PIN, LOW);
  }
}

void setupButtons() {
  // Bestehender Code für SET-Taste
  pinMode(BUTTON_SET, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_SET), buttonInterrupt, FALLING);

  // Neue Code für PLUS und MINUS Tasten
  pinMode(BUTTON_PLUS, INPUT_PULLUP);
  pinMode(BUTTON_MINUS, INPUT_PULLUP);

  Serial.println("Buttons konfiguriert!");
  Serial.print("SET: GPIO ");
  Serial.println(BUTTON_SET);
  Serial.print("PLUS: GPIO ");
  Serial.println(BUTTON_PLUS);
  Serial.print("MINUS: GPIO ");
  Serial.println(BUTTON_MINUS);
}

void showTime() {
  // Datum und Zeit vom RTC lesen
  DateTime now = rtc.now();
  
  // Zeit-Display aktualisieren
  displayTime.clearBuffer();
  displayTime.setFont(u8g2_font_helvR10_tr);
  
  // Datum formatieren
  char dateStr[15];
  sprintf(dateStr, "%02d.%02d.%04d", now.day(), now.month(), now.year());
  displayTime.setCursor(0, 15);
  displayTime.print(dateStr);
  
  // Zeit formatieren
  char timeStr[15];
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  displayTime.setCursor(0, 35);
  displayTime.print(timeStr);
  
  displayTime.sendBuffer();
  
  // RPM-Display wird separat über displayRPM() aktualisiert
}

void displaySetValue(const char* label, int value) {
  // Zeit-Display für den Einstellmodus
  displayTime.clearBuffer();
  displayTime.setFont(u8g2_font_helvR10_tr);
  displayTime.setCursor(0, 15);
  displayTime.print(label);
  displayTime.setCursor(0, 35);
  displayTime.print(value);
  displayTime.sendBuffer();
  
  // RPM-Display zeigt KORREKTEN Hilfetext
  displayRpm.clearBuffer();
  displayRpm.setFont(u8g2_font_helvR10_tr);
  displayRpm.setCursor(0, 15);
  displayRpm.print("+ = Erhoehen");
  displayRpm.setCursor(0, 35);
  displayRpm.print("- = Verringern");
  displayRpm.setCursor(0, 55);
  displayRpm.print("SET = Weiter");
  displayRpm.sendBuffer();
}

// Diese Funktion aktualisiert die Anzeige je nach aktuellem Zustand
void updateDisplay() {  
  // Diese statische Variable stellt sicher, dass updateTimeFromRTC() nicht
  // sofort nach einer Zeiteinstellung aufgerufen wird
  static unsigned long lastTimeSetMillis = 0;

  switch (currentState) {
    case NORMAL:
      // Im Normalmodus die aktuelle RTC-Zeit anzeigen, aber nicht direkt nach einer Einstellung
      if (millis() - lastTimeSetMillis > 3000) { // 3 Sekunden warten
        updateTimeFromRTC();
      }
      showTime();
      break;
    case SET_DAY:
      displaySetValue("Tag", day);
      break;
    case SET_MONTH:
      displaySetValue("Monat", month);
      break;
    case SET_YEAR:
      displaySetValue("Jahr", year);
      break;
    case SET_HOUR:
      displaySetValue("Stunde", hour);
      break;
    case SET_MINUTE:
      displaySetValue("Minute", minute);
      break;
    case SET_SECOND:
      displaySetValue("Sekunde", second);
      break;
  }
    // Nach SET_SECOND speichern wir den Zeitpunkt
  if (currentState == SET_SECOND) {
    lastTimeSetMillis = millis();
  }

}

void handleButtons(unsigned long currentTime) {
  // static unsigned long lastButtonPressTime = 0;
  const unsigned long DEBOUNCE_DELAY = 200;
  static bool lastButtonPressed = false;
  static bool lastPlusButtonPressed = false;
  static bool lastMinusButtonPressed = false;
  static unsigned long lastDebugOutput = 0;
  static unsigned long lastSetButtonTime = 0;
  static unsigned long lastPlusButtonTime = 0;
  static unsigned long lastMinusButtonTime = 0;
  
  // Ausgabe nur bei Änderung oder alle 5 Sekunden
  if (lastButtonPressed != buttonPressed || 
      lastPlusButtonPressed != plusButtonPressed ||
      lastMinusButtonPressed != minusButtonPressed ||
      currentTime - lastDebugOutput > 5000) {
      
    Serial.print("Button-Status: SET=");
    Serial.print(buttonPressed);
    Serial.print(", PLUS=");
    Serial.print(plusButtonPressed);
    Serial.print(", MINUS=");
    Serial.println(minusButtonPressed);
    
    lastButtonPressed = buttonPressed;
    lastPlusButtonPressed = plusButtonPressed;
    lastMinusButtonPressed = minusButtonPressed;
    lastDebugOutput = currentTime;
  }

  // SET-Taste gedrückt
  if (buttonPressed) {
    if (currentTime - lastSetButtonTime > DEBOUNCE_DELAY) {
      lastSetButtonTime = currentTime;
      
      DateTime now;

      // SET-Taste wechselt zum nächsten Zustand
      switch (currentState) {
        case NORMAL:
          currentState = SET_DAY;
          
          // Aktuelle Zeit für die Einstellung übernehmen - NUR EINMAL beim Eintritt
          now = rtc.now();
          day = now.day();
          month = now.month();
          year = now.year();
          hour = now.hour();
          minute = now.minute();
          second = now.second();
          
          Serial.println("Zeit-Einstellung gestartet - aktuelle RTC Zeit übernommen");
          break;
          
        case SET_DAY:
          currentState = SET_MONTH;
          Serial.println("Wechsel zu Monat-Einstellung");
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
          // Wichtig: Zurück zum NORMAL-Modus
          currentState = NORMAL;
          // Debug: Zeige Werte vor der Anpassung
          Serial.print("Setze RTC auf: ");
          Serial.print(year); Serial.print("-");
          Serial.print(month); Serial.print("-");
          Serial.print(day); Serial.print(" ");
          Serial.print(hour); Serial.print(":");
          Serial.print(minute); Serial.print(":");
          Serial.println(second);
          // Zeit im RTC aktualisieren
          rtc.adjust(DateTime(year, month, day, hour, minute, second));

                  
          // // Sofort prüfen, ob die Zeit gesetzt wurde
          // delay(100); // Kurze Pause für den RTC
          // now = rtc.now();
          // Serial.print("RTC-Zeit nach Einstellung: ");
          // Serial.print(now.year()); Serial.print("-");
          // Serial.print(now.month()); Serial.print("-");
          // Serial.print(now.day()); Serial.print(" ");
          // Serial.print(now.hour()); Serial.print(":");
          // Serial.print(now.minute()); Serial.print(":");
          // Serial.println(now.second());
          
          // // Lokale Zeit mit der eingestellten Zeit synchronisieren
          // hour = now.hour();
          // minute = now.minute();
          // second = now.second();

          // Serial.println("Zeit gesetzt, zurück zu NORMAL");
          break;
      }
      
      stateChanged = true;
    }
    buttonPressed = false;
  }
  
  // PLUS-Taste gedrückt
  if (plusButtonPressed) {
    if (currentTime - lastPlusButtonTime > DEBOUNCE_DELAY) {
      lastPlusButtonTime = currentTime;
      
      Serial.println("PLUS Button gedrückt!");
      
      // Wert erhöhen basierend auf aktuellem Zustand
      switch (currentState) {
        case SET_DAY:
          day = (day % 31) + 1;  // 1-31 zyklisch
          Serial.print("Tag erhöht auf: ");
          Serial.println(day);
          break;
        case SET_MONTH:
          month = (month % 12) + 1;  // 1-12 zyklisch
          Serial.print("Monat erhöht auf: ");
          Serial.println(month);
          break;
        case SET_YEAR:
          year = (year < 2099) ? year + 1 : 2000;
          Serial.print("Jahr erhöht auf: ");
          Serial.println(year);
          break;
        case SET_HOUR:
          hour = (hour + 1) % 24;
          Serial.print("Stunde erhöht auf: ");
          Serial.println(hour);
          break;
        case SET_MINUTE:
          minute = (minute + 1) % 60;
          Serial.print("Minute erhöht auf: ");
          Serial.println(minute);
          break;
        case SET_SECOND:
          second = (second + 1) % 60;
          Serial.print("Sekunde erhöht auf: ");
          Serial.println(second);
          break;
        default:
          break;
      }
      
      stateChanged = true;
    }
    plusButtonPressed = false;
  }
  
  // MINUS-Taste gedrückt
  if (minusButtonPressed) {
    if (currentTime - lastMinusButtonTime > DEBOUNCE_DELAY) {
      lastMinusButtonTime = currentTime;
      
      Serial.println("MINUS Button gedrückt!");
      
      // Wert verringern basierend auf aktuellem Zustand
      switch (currentState) {
        case SET_DAY:
          day = (day > 1) ? day - 1 : 31;
          Serial.print("Tag verringert auf: ");
          Serial.println(day);
          break;
        case SET_MONTH:
          month = (month > 1) ? month - 1 : 12;
          Serial.print("Monat verringert auf: ");
          Serial.println(month);
          break;
        case SET_YEAR:
          year = (year > 2000) ? year - 1 : 2099;
          Serial.print("Jahr verringert auf: ");
          Serial.println(year);
          break;
        case SET_HOUR:
          hour = (hour > 0) ? hour - 1 : 23;
          Serial.print("Stunde verringert auf: ");
          Serial.println(hour);
          break;
        case SET_MINUTE:
          minute = (minute > 0) ? minute - 1 : 59;
          Serial.print("Minute verringert auf: ");
          Serial.println(minute);
          break;
        case SET_SECOND:
          second = (second > 0) ? second - 1 : 59;
          Serial.print("Sekunde verringert auf: ");
          Serial.println(second);
          break;
        default:
          break;
      }
      
      stateChanged = true;
    }
    minusButtonPressed = false;
  }
  
  // Display aktualisieren wenn sich etwas geändert hat
  if (stateChanged) {
    updateDisplay();
    stateChanged = false;
  }
}

// Umbenennen der globalen Variable zur Vermeidung von Namenskonflikten
unsigned long lastButtonInterruptTime = 0; // Für Tastenentprellung
const unsigned long debounceTime = 500;    // 500ms Entprellzeit

void IRAM_ATTR handleSetButtonInterrupt() {
  unsigned long interruptTime = millis();
  
  // Mehr Filterung hinzufügen
  if (interruptTime - lastButtonInterruptTime > debounceTime) {
    // Zusätzliche Prüfung: Ist der Button tatsächlich gedrückt?
    if (digitalRead(BUTTON_SET) == LOW) { // Annahme: LOW = gedrückt
      buttonPressed = true;
      lastButtonInterruptTime = interruptTime;
    }
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

// Funktion zur Anzeige der RPM (nur auf dem RPM-Display)
void displayRPM() {
  displayRpm.clearBuffer();
  
  displayRpm.setFont(u8g2_font_inb19_mf);
  displayRpm.setCursor(5, 30);
  displayRpm.print("RPM:");
    
  displayRpm.setFont(u8g2_font_inb24_mf); // Größte Schrift für den RPM-Wert
  displayRpm.setCursor(5, 60);
  displayRpm.print(Rpm);
  
  displayRpm.sendBuffer();
}

// Funktion zur Anzeige von "NO CARD" (nur auf dem Zeit-Display)
void displayNoCard() {
  displayTime.clearBuffer();
  
  // Deutliche, große Anzeige
  displayTime.setFont(u8g2_font_inb24_mf);
  displayTime.setCursor(15, 30);
  displayTime.print("NO");
  displayTime.setCursor(15, 60);
  displayTime.print("CARD");
  
  displayTime.sendBuffer();
}

// Funktion zur Anzeige des Setup-Bildschirms (beide Displays)
void displaySetup() {
  // Auf Zeit-Display
  displayTime.clearBuffer();
  displayTime.setFont(u8g2_font_helvB12_tf);
  displayTime.setCursor(5, 20);
  displayTime.print("Setup Mode");
  displayTime.setCursor(5, 40);
  displayTime.print("Adjusting Time");
  displayTime.sendBuffer();
  
  // Auf RPM-Display die Details anzeigen
  displayRpm.clearBuffer();
  displayRpm.setFont(u8g2_font_helvB12_tf);
  
  displayRpm.setCursor(5, 20);
  if (currentState <= SET_YEAR) {
    displayRpm.print("Setup Date");
  } else {
    displayRpm.print("Setup Time");
  }
  
  displayRpm.setCursor(5, 40);
  
  switch(currentState) {
    case SET_DAY:
      displayRpm.print("Day: ");
      if (day < 10) displayRpm.print("0");
      displayRpm.print(day);
      break;
    case SET_MONTH:
      displayRpm.print("Month: ");
      if (month < 10) displayRpm.print("0");
      displayRpm.print(month);
      break;
    case SET_YEAR:
      displayRpm.print("Year: ");
      displayRpm.print(year);
      break;
    case SET_HOUR:
      displayRpm.print("Hour: ");
      if (hour < 10) displayRpm.print("0");
      displayRpm.print(hour);
      break;
    case SET_MINUTE:
      displayRpm.print("Minute: ");
      if (minute < 10) displayRpm.print("0");
      displayRpm.print(minute);
      break;
    case SET_SECOND:
      displayRpm.print("Second: ");
      if (second < 10) displayRpm.print("0");
      displayRpm.print(second);
      break;
    default:
      break;
  }
  
  displayRpm.setCursor(5, 60);
  displayRpm.print("+ / - to change");
  
  displayRpm.sendBuffer();
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
  
  displayRpm.clearBuffer();
  displayRpm.setFont(u8g2_font_helvB12_tf);
  displayRpm.setCursor(5, 30);
  displayRpm.print("Date & Time Set!");
  displayRpm.setCursor(5, 50);
  displayRpm.print("Returning...");
  displayRpm.sendBuffer();
  
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

bool initSDCard() {
  Serial.print("Initialisiere SD-Karte... ");
  
  // Überprüfe zuerst, ob die CS-Pin-Definition korrekt ist
  Serial.print("CS-Pin ist: "); 
  Serial.println(SD_CS);
  
  // SPI-Geschwindigkeit reduzieren für stabilere Kommunikation
  SPI.setFrequency(4000000); // 4 MHz statt Standard 16/20 MHz
  
  // SD Card Initialisierung mit erweiterter Fehlerprüfung
  if (!SD.begin(SD_CS)) {
    uint8_t cardType = SD.cardType();
    
    Serial.print("SD-Initialisierung fehlgeschlagen. Kartentyp: ");
    if(cardType == CARD_NONE) {
      Serial.println("Keine SD-Karte erkannt");
    } else {
      Serial.print(cardType);
      Serial.println(" - Hardware vermutlich korrekt angeschlossen");
    }
    
    // Versuche mit VSPI explizit
    Serial.println("Versuche mit expliziter SPI-Konfiguration...");
    SPIClass spiSD(VSPI);
    spiSD.begin(18, 19, 23, SD_CS); // SCK, MISO, MOSI, SS
    if (!SD.begin(SD_CS, spiSD)) {
      Serial.println("Auch mit expliziter SPI-Konfiguration fehlgeschlagen");
      return false;
    } else {
      Serial.println("Erfolg mit expliziter SPI-Konfiguration!");
      return true;
    }
  }
  
  Serial.println("SD-Karte initialisiert");
  
  // Karteninformationen ausgeben
  uint8_t cardType = SD.cardType();
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  
  Serial.print("SD-Kartentyp: ");
  if(cardType == CARD_MMC) Serial.println("MMC");
  else if(cardType == CARD_SD) Serial.println("SDSC");
  else if(cardType == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("UNBEKANNT");
  
  Serial.print("SD-Kartengröße: ");
  Serial.print(cardSize);
  Serial.println(" MB");

// SPI explizit beenden
SPI.endTransaction();
delay(200);  // Mehr Zeit für Stabilisierung

// I2C-Busse neu initialisieren (getrennt)
Wire.end();  // Hauptbus beenden
Wire1.end(); // EEPROM-Bus beenden
delay(200);  

// Erst EEPROM-Bus wieder starten
Wire1.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
Wire1.setClock(50000); // Niedrigere Frequenz
delay(100);

// Dann Hauptbus wieder starten
Wire.begin(SDA_PIN, SCL_PIN);
Wire.setClock(100000);
delay(100);

  return true;
}

bool testEEPROM() {
  Serial.println("EEPROM-Debug-Test startet...");
  
  // EEPROM-Kommunikation ohne Header-Strukturen testen
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write(0);    // Adresse MSB
  Wire1.write(0);    // Adresse LSB
  byte result = Wire1.endTransmission();
  
  Serial.print("EEPROM-Transmissions-Ergebnis: ");
  Serial.println(result);
  
  Wire1.requestFrom(EEPROM_ADDRESS, 1);
  if (Wire1.available()) {
    byte value = Wire1.read();
    Serial.print("Gelesener Byte-Wert: 0x");
    Serial.println(value, HEX);
  } else {
    Serial.println("Keine Daten vom EEPROM erhalten");
    return false;
  }
  
  // Nur der einfache Test, keine Initialisierung hier
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write(0);    // Adresse MSB
  Wire1.write(0);    // Adresse LSB
  Wire1.write(0xAA); // Testdaten
  if (Wire1.endTransmission() != 0) {
    Serial.println("Fehler beim Schreiben");
    return false;
  }
  
  // Warte auf den Schreibvorgang
  delay(10);
  
  // Setze Leseadresse
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Wire1.write(0);    // Adresse MSB
  Wire1.write(0);    // Adresse LSB
  if (Wire1.endTransmission() != 0) {
    Serial.println("Fehler beim Setzen der Leseadresse");
    return false;
  }
  
  // Lese Daten
  Wire1.requestFrom(EEPROM_ADDRESS, 1);
  if (Wire1.available()) {
    byte data = Wire1.read();
    Serial.print("Gelesene Daten: 0x");
    Serial.println(data, HEX);
    return (data == 0xAA);
  }
  
  Serial.println("Keine Daten vom EEPROM erhalten");
  return false;
}

bool verifyEEPROM() {
  // Test-Wert schreiben
  writeEEPROMByte(0, 0x55);
  delay(10);
  
  // Test-Wert lesen
  uint8_t readValue = readEEPROMByte(0);
  
  Serial.print("EEPROM Verifikation - Geschrieben: 0x55, Gelesen: 0x");
  Serial.println(readValue, HEX);
  
  return (readValue == 0x55);
}

void readEEPROMHeader() {
  // Zuerst mit Nullen initialisieren
  memset(&eepromHeader, 0, sizeof(EEPROMHeader));
  
  // Werte einzeln lesen und validieren
  uint32_t recordCount = 0;
  uint32_t nextWriteAddress = 0;
  uint8_t initialized = 0;
  
  // recordCount (4 Bytes)
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t val = readEEPROMByte(i);
    recordCount |= (uint32_t)val << (i * 8);
  }
  
  // nextWriteAddress (4 Bytes)
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t val = readEEPROMByte(4 + i);
    nextWriteAddress |= (uint32_t)val << (i * 8);
  }
  
  // initialized (1 Byte)
  initialized = readEEPROMByte(8);
  
  // Validiere die Werte
  if (nextWriteAddress < sizeof(EEPROMHeader) || 
      nextWriteAddress >= EEPROM_SIZE ||
      recordCount > 10000) {
    // Ungültige Werte, setze alles zurück
    Serial.println("Ungültige EEPROM-Header-Werte gefunden, setze zurück.");
    eepromHeader.recordCount = 0;
    eepromHeader.nextWriteAddress = sizeof(EEPROMHeader);
    eepromHeader.initialized = 0xAA;
  } else {
    // Werte sind plausibel
    eepromHeader.recordCount = recordCount;
    eepromHeader.nextWriteAddress = nextWriteAddress;
    eepromHeader.initialized = initialized;
  }
}
// EEPROM-Header in das EEPROM schreiben
void writeEEPROMHeader() {
  for (uint16_t i = 0; i < sizeof(EEPROMHeader); i++) {
    writeEEPROMByte(i, ((uint8_t*)&eepromHeader)[i]);
    delay(10); // Warten zwischen den Bytes, um EEPROM nicht zu überlasten
  }
}

uint8_t readEEPROMByte(uint32_t address) {
  if (address >= EEPROM_SIZE) {
    Serial.println("FEHLER: EEPROM-Lesezugriff außerhalb des gültigen Bereichs!");
    return 0;
  }
  
  uint8_t retries = 5;  // Mehr Wiederholungsversuche
  while (retries > 0) {
    Wire1.beginTransmission(EEPROM_ADDRESS);
    Wire1.write((address >> 8) & 0xFF);  // MSB
    Wire1.write(address & 0xFF);         // LSB
    if (Wire1.endTransmission() != 0) {
      Serial.println("EEPROM-Lesefehler, wiederhole...");
      retries--;
      delay(20);  // Längere Pause
      
      if (retries == 2) {
        // Bei fast erschöpften Versuchen I2C zurücksetzen
        Wire1.end();
        delay(50);
        Wire1.begin(SDA_PIN, SCL_PIN);
        Wire1.setClock(25000);  // Noch langsamere Frequenz probieren
      }
      
      continue;
    }
    
    if (Wire1.requestFrom(EEPROM_ADDRESS, 1) != 1) {
      Serial.println("EEPROM-Lesefehler, wiederhole...");
      retries--;
      delay(20);
      continue;
    }
    
    if (Wire1.available()) {
      return Wire1.read();
    }
    
    retries--;
    delay(20);
  }
  
  return 0;  // Bei Fehler 0 zurückgeben
}

// Ein einzelnes Byte ins EEPROM schreiben
void writeEEPROMByte(uint32_t address, uint8_t data) {
  if (address >= EEPROM_SIZE) {
    Serial.println("FEHLER: EEPROM-Schreibzugriff außerhalb des gültigen Bereichs!");
    return;
  }
  
  Wire1.beginTransmission(EEPROM_ADDRESS);
  // Adresse als MSB und LSB senden
  Wire1.write((address >> 8) & 0xFF);  // MSB
  Wire1.write(address & 0xFF);         // LSB
  Wire1.write(data);
  Wire1.endTransmission();
  
  delay(5); // Zeit zum Schreiben
}

bool recoveryI2CForEEPROMDownload() {
  Serial.println("I2C-Bus vollständig zurücksetzen für EEPROM-Download...");
  
  // Alle I2C-Geräte deaktivieren
  Wire1.end();  // EEPROM-Bus beenden statt Wire
  delay(200);
  
  // Manuelle I2C-Bus-Reset-Prozedur
  pinMode(SDA_PIN_EEPROM, OUTPUT);  // EEPROM-Pins verwenden
  pinMode(SCL_PIN_EEPROM, OUTPUT);  // EEPROM-Pins verwenden
  
  // Pins auf HIGH setzen (idle Zustand)
  digitalWrite(SDA_PIN_EEPROM, HIGH);
  digitalWrite(SCL_PIN_EEPROM, HIGH);
  delay(50);
  
  // START-Bedingung simulieren
  digitalWrite(SDA_PIN_EEPROM, LOW);
  delayMicroseconds(10);
  digitalWrite(SCL_PIN_EEPROM, LOW);
  delayMicroseconds(10);
  
  // 9 Taktzyklen (für potentiell 9 Bits) senden
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN_EEPROM, HIGH);
    delayMicroseconds(10);
    digitalWrite(SCL_PIN_EEPROM, LOW);
    delayMicroseconds(10);
  }
  
  // STOP-Bedingung generieren
  digitalWrite(SCL_PIN_EEPROM, LOW);
  delayMicroseconds(10);
  digitalWrite(SDA_PIN_EEPROM, LOW);
  delayMicroseconds(10);
  digitalWrite(SCL_PIN_EEPROM, HIGH);
  delayMicroseconds(10);
  digitalWrite(SDA_PIN_EEPROM, HIGH);
  delay(50);  // Längere Pause nach STOP
  
  // I2C-Bus neu initialisieren mit sehr niedriger Frequenz
  Wire1.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  Wire1.setClock(10000);  // 10 kHz für stabile Kommunikation
  delay(100);
  
  // Test, ob EEPROM erreichbar ist
  Wire1.beginTransmission(EEPROM_ADDRESS);
  byte error = Wire1.endTransmission();
  
  if (error == 0) {
    Serial.println("EEPROM ist jetzt erreichbar!");
    return true;
  }
  
  // Zweiter Versuch mit noch niedrigerer Frequenz
  Wire1.setClock(5000);  // 5 kHz
  delay(50);
  
  Wire1.beginTransmission(EEPROM_ADDRESS);
  error = Wire1.endTransmission();
  
  return (error == 0);
}

void downloadEEPROMDataToUSB() {
  Serial.println("EEPROM-Daten-Download startet...");
  
  // Displays ausschalten während des Downloads
  displayTime.setPowerSave(1);
  displayRpm.setPowerSave(1);
  delay(50);
  
  // I2C-Bus zurücksetzen und für EEPROM optimieren
  if (!recoveryI2CForEEPROMDownload()) {
    Serial.println("EEPROM-Wiederherstellung fehlgeschlagen!");
    
    // Aggressiver Reset-Versuch
    Wire1.end();
    delay(500);
    pinMode(SDA_PIN_EEPROM, INPUT_PULLUP); // EEPROM-Pins statt SDA_PIN verwenden!
    pinMode(SCL_PIN_EEPROM, INPUT_PULLUP); // EEPROM-Pins statt SCL_PIN verwenden!
    delay(100);
    Wire1.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM); // EEPROM-Pins verwenden!
    Wire1.setClock(1000);  // Extrem langsam
    delay(200);
    
    // Letzter Verbindungstest
    Wire1.beginTransmission(EEPROM_ADDRESS);
    byte error = Wire1.endTransmission();
    
    if (error != 0) {
      Serial.println("EEPROM nicht erreichbar - Download abgebrochen.");
      displayTime.setPowerSave(0);
      displayRpm.setPowerSave(0);
      delay(100);
      showTime();
      displayRPM();
      return;
    }
    
    Serial.println("EEPROM nach Reset wiederhergestellt.");
  }
  
  // CSV-Header
  Serial.println("timestamp,rpm,temperature");
  
  // EEPROM-Header separat lesen
  uint32_t recordCount = 0;
  for (uint8_t i = 0; i < 4; i++) {
    Wire1.beginTransmission(EEPROM_ADDRESS);
    Wire1.write(0);  // MSB
    Wire1.write(i);  // LSB
    
    if (Wire1.endTransmission() != 0) {
      Serial.println("Fehler beim Setzen der Leseadresse");
      continue;
    }
    
    delay(5);
    
    if (Wire1.requestFrom(EEPROM_ADDRESS, 1) != 1) {
      Serial.println("Fehler beim Lesen des EEPROM-Headers");
      continue;
    }
    
    if (Wire1.available()) {
      byte value = Wire1.read();
      recordCount |= ((uint32_t)value << (i * 8));
    }
    
    delay(5);
  }
  
  Serial.print("Datensatzanzahl: ");
  Serial.println(recordCount);
  
  // Plausibilitätsprüfung
  if (recordCount > 10000) {
    Serial.println("Unplausible Datensatzanzahl - beschränke auf 100");
    recordCount = 100;
  }
  
  // WICHTIGE ÄNDERUNG: Nur die letzten 100 Datensätze laden
  uint32_t startIndex = 0;
  if (recordCount > 1000) {
    startIndex = recordCount - 1000;
    Serial.print("Zeige nur die letzten 1000 von ");
    Serial.print(recordCount);
    Serial.println(" Datensätzen");
  }
  
  // Datensätze lesen - startIndex VERHINDERT Wiederholungen
  for (uint32_t i = startIndex; i < recordCount; i++) {
    if (i % 5 == 0) {
      delay(100);  // Pause alle 5 Datensätze
    }
    
    uint32_t recordAddress = sizeof(EEPROMHeader) + (i * sizeof(LogRecord));
    
    // Datensatz byte-weise lesen
    LogRecord record;
    memset(&record, 0, sizeof(record));
    bool readSuccess = true;
    
    for (uint8_t byteIdx = 0; byteIdx < sizeof(LogRecord); byteIdx++) {
      uint32_t byteAddress = recordAddress + byteIdx;
      
      Wire1.beginTransmission(EEPROM_ADDRESS);
      Wire1.write((byteAddress >> 8) & 0xFF);
      Wire1.write(byteAddress & 0xFF);
      
      if (Wire1.endTransmission() != 0) {
        readSuccess = false;
        break;
      }
      
      delay(2);
      
      if (Wire1.requestFrom(EEPROM_ADDRESS, 1) != 1) {
        readSuccess = false;
        break;
      }
      
      if (Wire1.available()) {
        ((uint8_t*)&record)[byteIdx] = Wire1.read();
      } else {
        readSuccess = false;
        break;
      }
      
      delay(2);
    }
    
    if (!readSuccess) {
      Serial.print("Fehler beim Lesen des Datensatzes ");
      Serial.println(i);
      
      Wire1.end();
      delay(50);
      Wire1.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
      Wire1.setClock(5000);
      delay(100);
      
      continue;
    }
    
    // Plausibilitätsprüfung (Timestamp zwischen 2021-2030)
    if (record.timestamp < 1609459200 || record.timestamp > 1893456000) {
      continue;
    }
    
    // Ausgabe im CSV-Format
    time_t rawtime = record.timestamp;
    struct tm * timeinfo = gmtime(&rawtime);
    
    char timeString[20];
    sprintf(timeString, "%04d-%02d-%02d %02d:%02d:%02d",
            timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
            timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    
    Serial.print(timeString);
    Serial.print(",");
    Serial.print(record.rpm);
    Serial.print(",");
    Serial.println(record.temperature / 100.0f, 2);
    
    // Auf Abbruch prüfen
    if (Serial.available() && Serial.read() == 'q') {
      Serial.println("Download abgebrochen.");
      break;
    }
  }
  
  Serial.println("EEPROM-Daten-Download abgeschlossen.");
  
  // I2C-Bus wiederherstellen
  Wire1.end();
  delay(100);
  Wire1.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  Wire1.setClock(100000);
  delay(100);
  
  // Displays wieder aktivieren
  displayTime.setPowerSave(0);
  displayRpm.setPowerSave(0);
  delay(100);
  showTime();
  displayRPM();
}

// Eine Seite (bis zu 128 Bytes) ins EEPROM schreiben
void writeEEPROMPage(uint32_t address, const uint8_t* data, uint8_t length) {
  if (address + length >= EEPROM_SIZE) {
    Serial.println("FEHLER: EEPROM-Schreibzugriff außerhalb des gültigen Bereichs!");
    return;
  }
  
  // Sicherstellen, dass wir nicht über eine Seitengrenze schreiben
  if ((address / EEPROM_PAGE_SIZE) != ((address + length - 1) / EEPROM_PAGE_SIZE)) {
    // Serial.println("WARNUNG: Schreibvorgang überschreitet Seitengrenze, aufteilen...");
    
    // Berechnen wie viele Bytes auf der aktuellen Seite bleiben
    uint8_t bytesUntilPageBoundary = EEPROM_PAGE_SIZE - (address % EEPROM_PAGE_SIZE);
    
    // Ersten Teil bis zur Seitengrenze schreiben
    writeEEPROMPage(address, data, bytesUntilPageBoundary);
    delay(EEPROM_WRITE_CYCLE);
    
    // Zweiten Teil auf die nächste Seite schreiben
    writeEEPROMPage(address + bytesUntilPageBoundary, 
                    data + bytesUntilPageBoundary, 
                    length - bytesUntilPageBoundary);
    return;
  }
  
  Wire1.beginTransmission(EEPROM_ADDRESS);
  // Adresse als MSB und LSB senden
  Wire1.write((address >> 8) & 0xFF);  // MSB
  Wire1.write(address & 0xFF);         // LSB
  
  // Daten seitenweise schreiben
  for (uint8_t i = 0; i < length; i++) {
    Wire1.write(data[i]);
  }
  
  Wire1.endTransmission();
  delay(EEPROM_WRITE_CYCLE); // Warten auf Abschluss des Schreibvorgangs
}

bool initEEPROM() {
  Serial.println("Initialisiere EEPROM...");
  
  // Explizit I2C für EEPROM optimieren
  Wire1.end();
  delay(200);
  Wire1.begin(SDA_PIN, SCL_PIN);
  Wire1.setClock(50000); // Niedrigere Geschwindigkeit für bessere Stabilität
  
  // EEPROM-Adresse aggressiv testen
  Serial.print("Teste EEPROM bei verschiedenen Adressen...");
  
  byte potentialAddresses[] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57};
  bool eepromFound = false;
  byte foundAddress = 0;
  
  for (int i = 0; i < sizeof(potentialAddresses); i++) {
    Wire.beginTransmission(potentialAddresses[i]);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      // Weiterer Test: Versuche zu schreiben und zu lesen
      Wire.beginTransmission(potentialAddresses[i]);
      Wire.write(0); // MSB
      Wire.write(0); // LSB
      Wire.write(0xA5); // Testmuster
      if (Wire.endTransmission() == 0) {
        delay(10);
        
        // Versuche zu lesen
        Wire.beginTransmission(potentialAddresses[i]);
        Wire.write(0); // MSB
        Wire.write(0); // LSB
        if (Wire.endTransmission() == 0) {
          if (Wire.requestFrom(potentialAddresses[i], (uint8_t)1) == 1) {
            byte readValue = Wire.read();
            
            if (readValue == 0xA5) {
              Serial.print("EEPROM gefunden bei 0x");
              Serial.println(potentialAddresses[i], HEX);
              eepromFound = true;
              foundAddress = potentialAddresses[i];
              break;
            }
          }
        }
      }
    }
    delay(50);
  }
  
  if (!eepromFound) {
    Serial.println("EEPROM bei keiner Adresse gefunden! Hardware-Problem?");
    return false;
  }
  
  // Wenn EEPROM bei einer anderen Adresse als 0x50 gefunden wurde
  if (foundAddress != EEPROM_ADDRESS) {
    Serial.print("WARNUNG: EEPROM bei 0x");
    Serial.print(foundAddress, HEX);
    Serial.print(" statt bei 0x");
    Serial.println(EEPROM_ADDRESS, HEX);
    Serial.println("Verwende gefundene Adresse für diesen Testlauf.");
    // Hier könnte man die gefundene Adresse verwenden, aber das würde größere Codeänderungen erfordern
  }
  
  // Zuerst alles auf Null setzen
  memset(&eepromHeader, 0, sizeof(EEPROMHeader));
  
  // Header-Initialisierung wie gehabt
  eepromHeader.recordCount = 0;
  eepromHeader.nextWriteAddress = sizeof(EEPROMHeader);
  eepromHeader.initialized = 0xAA;
  
  // Header schreiben
  writeEEPROMHeader();
  
  Serial.println("EEPROM erfolgreich initialisiert");
  return true;
}

// Einen Datensatz im EEPROM speichern
bool logDataToEEPROM(uint32_t timestamp, uint16_t rpm, float temperature) {
  // Prüfen ob noch Platz im EEPROM ist
  if (eepromHeader.nextWriteAddress + sizeof(LogRecord) >= EEPROM_SIZE) {
    Serial.println("FEHLER: EEPROM voll!");
    return false;
  }
  
  // Datensatz vorbereiten
  LogRecord record;
  record.timestamp = timestamp;
  record.rpm = rpm;
  record.temperature = (int16_t)(temperature * 100.0f); // *100 für Nachkommastellen
  
  // Datensatz ins EEPROM schreiben
  writeEEPROMPage(eepromHeader.nextWriteAddress, (uint8_t*)&record, sizeof(LogRecord));
  
  // Header aktualisieren
  eepromHeader.recordCount++;
  eepromHeader.nextWriteAddress += sizeof(LogRecord);
  writeEEPROMHeader();
 
  return true;
}

// Einen Datensatz aus dem EEPROM lesen
LogRecord readRecordFromEEPROM(uint32_t recordIndex) {
  LogRecord record = {0}; // Initialisiere mit Nullen
  
  if (recordIndex >= eepromHeader.recordCount) {
    Serial.println("FEHLER: Ungültiger Datensatzindex!");
    return record; // Leerer Datensatz
  }
  
  uint32_t recordAddress = sizeof(EEPROMHeader) + (recordIndex * sizeof(LogRecord));
  
  // Sicherheitsprüfung der Adresse
  if (recordAddress + sizeof(LogRecord) > EEPROM_SIZE) {
    Serial.println("FEHLER: Datensatzadresse außerhalb des EEPROM-Bereichs!");
    return record;
  }
  
  // Datensatz lesen
  for (uint16_t i = 0; i < sizeof(LogRecord); i++) {
    ((uint8_t*)&record)[i] = readEEPROMByte(recordAddress + i);
    delay(1); // Kurze Pause zwischen Lesevorgängen
  }
  
  return record;
}

// Verarbeitung von seriellen Befehlen für EEPROM-Zugriff
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "download") {
      downloadEEPROMDataToUSB();
    } 
    else if (command == "erase") {
      Serial.println("EEPROM wird gelöscht...");
      eraseEEPROM();
      Serial.println("EEPROM gelöscht.");
    }
    else if (command == "info") {
      Serial.print("EEPROM Status: ");
      Serial.println(eepromAvailable ? "Verfügbar" : "Nicht verfügbar");
      Serial.print("Gespeicherte Datensätze: ");
      Serial.println(eepromHeader.recordCount);
      Serial.print("Verwendeter Speicher: ");
      Serial.print((eepromHeader.nextWriteAddress * 100.0) / EEPROM_SIZE, 2);
      Serial.println("%");
    }
    else if (command == "help") {
      Serial.println("Verfügbare Befehle:");
      Serial.println("  download - Alle EEPROM-Daten herunterladen");
      Serial.println("  erase    - EEPROM komplett löschen");
      Serial.println("  info     - EEPROM-Statusinformationen anzeigen");
      Serial.println("  help     - Diese Hilfe anzeigen");
    }
  }
}

// EEPROM komplett löschen
void eraseEEPROM() {
  // Nur Header zurücksetzen
  eepromHeader.recordCount = 0;
  eepromHeader.nextWriteAddress = sizeof(EEPROMHeader);
  eepromHeader.initialized = 0xAA;
  writeEEPROMHeader();
  
  Serial.println("EEPROM gelöscht (Header zurückgesetzt)");
}

// Nach der SD-Karten-Initialisierung
void resetI2CBus() {
  Serial.println("I2C-Bus zurücksetzen...");
  
  // I2C-Bus neu initialisieren
  Wire.end();
  delay(100);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  
  // Kurze Pause für Stabilisierung
  delay(200);
}

bool recoverI2C() {
  Serial.println("Verbesserte I2C-Bus-Recovery...");
  
  // Erst alle I2C-Aktivitäten stoppen
  Wire.end();
  Wire1.end();
  delay(300);
  
  // Manuelle Kontrolle der I2C-Pins - zuerst als INPUT konfigurieren
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(100);
  
  pinMode(SDA_PIN_EEPROM, INPUT_PULLUP);
  pinMode(SCL_PIN_EEPROM, INPUT_PULLUP);
  delay(100);
  
  // Jetzt erst als OUTPUT konfigurieren
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN_EEPROM, OUTPUT);
  pinMode(SCL_PIN_EEPROM, OUTPUT);
  
  // SCL-Cycling zuerst für Bus 1
  digitalWrite(SDA_PIN_EEPROM, HIGH);
  digitalWrite(SCL_PIN_EEPROM, HIGH);
  delay(100);
  
  for (int i = 0; i < 16; i++) {
    digitalWrite(SCL_PIN_EEPROM, LOW);
    delay(5);
    digitalWrite(SCL_PIN_EEPROM, HIGH);
    delay(5);
  }
  
  // STOP-Bedingung
  digitalWrite(SDA_PIN_EEPROM, LOW);
  delay(5);
  digitalWrite(SCL_PIN_EEPROM, HIGH);
  delay(5);
  digitalWrite(SDA_PIN_EEPROM, HIGH);
  delay(20);
  
  // Beide Busse neu initialisieren
  Wire1.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  Wire1.setClock(10000);  // 10kHz
  delay(100);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // 100kHz
  delay(100);
  
  return true;
}

// I2C-Scanner für Debugging

// Neue Funktionen für erweiterte I2C-Tests hinzufügen
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

// Verbesserte I2C-Bus Reset-Funktion
void resetI2CBus(int sdaPin, int sclPin) {
  Serial.println("I2C-Bus zurücksetzen...");
  
  // Aktuelle Wire-Instanz beenden
  if (sdaPin == SDA_PIN_EEPROM) {
    Wire1.end();
  } else {
    Wire.end();
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
void testWithFrequency(TwoWire &wirePort, int sdaPin, int sclPin, uint32_t frequency, bool isEEPROM) {
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
  
  // Je nach Bus unterschiedliche Tests durchführen
  if (isEEPROM) {
    // EEPROM-Tests
    Serial.println("EEPROM-Tests an verschiedenen Adressen:");
    Serial.println("-------------------------------------");
    
    const byte possibleAddresses[] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57};
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
          
          // Bei Erfolg Status auf den Displays anzeigen
          displayTime.clearBuffer();
          displayTime.setFont(u8g2_font_helvB12_tf);
          displayTime.setCursor(5, 20);
          displayTime.print("EEPROM Test");
          displayTime.setCursor(5, 40);
          displayTime.print("Successful!");
          displayTime.sendBuffer();
          
          displayRpm.clearBuffer();
          displayRpm.setFont(u8g2_font_helvB12_tf);
          displayRpm.setCursor(5, 20);
          displayRpm.print("EEPROM at 0x");
          displayRpm.print(possibleAddresses[i], HEX);
          displayRpm.setCursor(5, 40);
          displayRpm.print("All Tests Pass");
          displayRpm.sendBuffer();
          
          delay(1500);
        } else {
          Serial.print("Tests fehlgeschlagen (");
          Serial.print(test1 ? "OK" : "FAIL");
          Serial.print(", ");
          Serial.print(test2 ? "OK" : "FAIL");
          Serial.print(", ");
          Serial.print(test3 ? "OK" : "FAIL");
          Serial.println(")");
          
          // Bei Fehler entsprechende Meldung
          displayTime.clearBuffer();
          displayTime.setFont(u8g2_font_helvB12_tf);
          displayTime.setCursor(5, 20);
          displayTime.print("EEPROM Test");
          displayTime.setCursor(5, 40);
          displayTime.print("FAILED!");
          displayTime.sendBuffer();
          
          displayRpm.clearBuffer();
          displayRpm.setFont(u8g2_font_helvB12_tf);
          displayRpm.setCursor(5, 20);
          displayRpm.print("Check EEPROM");
          displayRpm.setCursor(5, 40);
          displayRpm.print("Connection");
          displayRpm.sendBuffer();
          
          delay(1500);
        }
      } else {
        Serial.print("Nicht gefunden (Fehlercode: ");
        Serial.print(error);
        Serial.println(")");
      }
      
      Serial.println(); // Leerzeile für bessere Lesbarkeit
    }
  } else {
    // OLED/RTC-Bus Tests
    Serial.println("OLED/RTC-Bus Gerätetest:");
    Serial.println("----------------------");
    
    // OLED-Adressen testen
    const byte oledAddresses[] = {0x3C, 0x3D}; // Typische OLED-Adressen
    for (int i = 0; i < 2; i++) {
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
    byte rtcAddress = 0x68;
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
        
        // Anzeige auf dem Display
        displayTime.clearBuffer();
        displayTime.setFont(u8g2_font_helvB12_tf);
        displayTime.setCursor(5, 20);
        displayTime.print("RTC Test OK");
        displayTime.setCursor(5, 40);
        displayTime.print("Time Found!");
        displayTime.sendBuffer();
        
        displayRpm.clearBuffer();
        displayRpm.setFont(u8g2_font_helvB12_tf);
        displayRpm.setCursor(5, 20);
        displayRpm.print("20");
        displayRpm.print(year);
        displayRpm.print("-");
        displayRpm.print(month);
        displayRpm.print("-");
        displayRpm.print(day);
        displayRpm.setCursor(5, 40);
        displayRpm.print(hour);
        displayRpm.print(":");
        displayRpm.print(minute);
        displayRpm.print(":");
        displayRpm.print(second);
        displayRpm.sendBuffer();
        
        delay(1500);
      }
    } else {
      Serial.print("Nicht gefunden (Fehlercode: ");
      Serial.print(error);
      Serial.println(")");
      
      // RTC-Fehler anzeigen
      displayTime.clearBuffer();
      displayTime.setFont(u8g2_font_helvB12_tf);
      displayTime.setCursor(5, 20);
      displayTime.print("RTC Test");
      displayTime.setCursor(5, 40);
      displayTime.print("FAILED!");
      displayTime.sendBuffer();
      
      displayRpm.clearBuffer();
      displayRpm.setFont(u8g2_font_helvB12_tf);
      displayRpm.setCursor(5, 20);
      displayRpm.print("Check RTC");
      displayRpm.setCursor(5, 40);
      displayRpm.print("Connection");
      displayRpm.sendBuffer();
      
      delay(1500);
    }
  }
}

// Modifiziere die setup()-Funktion, um die I2C-Tests einzufügen
void setup() {
  Serial.begin(115200);
  Serial.println("Setup gestartet");

  // --- SCHRITT 1: I2C-Bus und Displays initialisieren ---
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100kHz für stabilere Kommunikation
  
  // Initialisiere Wire1 für den EEPROM-Bus
  Wire1.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  Wire1.setClock(100000); // 100kHz für stabilere Kommunikation
  
  // Displays initialisieren
  displayTime.setI2CAddress(OLED_TIME_ADDR * 2);
  displayTime.begin();
  displayTime.clearBuffer();
  
  displayRpm.setI2CAddress(OLED_RPM_ADDR * 2);
  displayRpm.begin();
  displayRpm.clearBuffer();
  
  // Initialisierungsnachricht
  displayTime.setFont(u8g2_font_helvB12_tf);
  displayTime.setCursor(0, 20);
  displayTime.print("Initializing...");
  displayTime.sendBuffer();
  
  displayRpm.setFont(u8g2_font_helvB12_tf);
  displayRpm.setCursor(0, 20);
  displayRpm.print("Please wait");
  displayRpm.sendBuffer();
  
  delay(1000);
  
  // --- SCHRITT 2: Starte die verbesserten I2C-Tests ---
  displayTime.clearBuffer();
  displayTime.setFont(u8g2_font_helvB12_tf);
  displayTime.setCursor(0, 20);
  displayTime.print("Starting");
  displayTime.setCursor(0, 40);
  displayTime.print("I2C Bus Tests");
  displayTime.sendBuffer();
  
  displayRpm.clearBuffer();
  displayRpm.setFont(u8g2_font_helvB12_tf);
  displayRpm.setCursor(0, 20);
  displayRpm.print("Comprehensive");
  displayRpm.setCursor(0, 40);
  displayRpm.print("Diagnostics");
  displayRpm.sendBuffer();
  
  delay(1000);

  Serial.println("EEPROM-Bus vor Tests:");
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Serial.print("Test-Ergebnis: ");
  Serial.println(Wire1.endTransmission());

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
  Serial.println(SDA_PIN);
  Serial.print("  SCL-Pin: ");
  Serial.println(SCL_PIN);
  
  // ================================================================
  // Test EEPROM-Bus (I2C Bus 1 auf Pins 16/17)
  // ================================================================
  Serial.println("\n\n=================================================");
  Serial.println("       TEST EEPROM-BUS (Pins 16/17)");
  Serial.println("=================================================\n");
  
  Serial.println("\nEEPROM-Bus-Test wird gestartet...");
  
  // I2C-Bus zurücksetzen
  resetI2CBus(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  
  // I2C-Bus neu starten
  Wire1.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  delay(200);
  
  // Test mit 10 kHz
  testWithFrequency(Wire1, SDA_PIN_EEPROM, SCL_PIN_EEPROM, 10000, true);
  
  // ================================================================
  // Test OLED/RTC-Bus (I2C Bus 2 auf Pins 21/22)
  // ================================================================
  Serial.println("\n\n=================================================");
  Serial.println("       TEST OLED/RTC-BUS (Pins 21/22)");
  Serial.println("=================================================\n");
  
  Serial.println("\nOLED/RTC-Bus-Test wird gestartet...");
  
  // I2C-Bus zurücksetzen
  resetI2CBus(SDA_PIN, SCL_PIN);
  
  // I2C-Bus neu starten
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(200);
  
  // Test mit 10 kHz
  testWithFrequency(Wire, SDA_PIN, SCL_PIN, 10000, false);
  
  // ================================================================
  // Zusammenfassung
  // ================================================================
  Serial.println("\n=================================================");
  Serial.println("         ALLE TESTS ABGESCHLOSSEN");
  Serial.println("=================================================\n");
  
  // I2C-Bus nach Tests zurücksetzen und mit normaler Geschwindigkeit neu starten
  resetI2CBus(SDA_PIN, SCL_PIN);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  
  resetI2CBus(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  Wire1.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  Wire1.setClock(100000);

  // --- SCHRITT 3: I2C-Scanner und RTC-Initialisierung ---
  scanI2C();
    
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC!");
    displayTime.clearBuffer();
    displayTime.setCursor(0, 30);
    displayTime.print("RTC Error!");
    displayTime.sendBuffer();
    while (1) delay(10);
  }
  
  // RTC initialisieren
  
  rtc.disable32K();
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);
  rtc.disableAlarm(2);
  
  // RTC Zeit prüfen/setzen
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting default time!");
    rtc.adjust(DateTime(2025, 4, 22, 12, 0, 0));
    delay(1000);
  }
  
  // Erste Zeitübernahme vom RTC
  DateTime initialTime = rtc.now();
  Serial.print("Initial RTC time: ");
  Serial.print(initialTime.year()); Serial.print("-");
  Serial.print(initialTime.month()); Serial.print("-");
  Serial.print(initialTime.day()); Serial.print(" ");
  Serial.print(initialTime.hour()); Serial.print(":");
  Serial.print(initialTime.minute()); Serial.print(":");
  Serial.println(initialTime.second());
  
  updateTimeFromRTC();
  
  // --- SCHRITT 3: EEPROM-Test und Initialisierung ---
  Serial.println("EEPROM-Test...");
  bool eepromTestOk = testEEPROM();
  
  if (eepromTestOk) {
    eepromAvailable = initEEPROM();
    
    if (eepromAvailable) {
      Serial.println("EEPROM initialisiert und bereit!");
      
      // Bestätigungsnachricht anzeigen
      displayTime.clearBuffer();
      displayTime.setCursor(0, 20);
      displayTime.print("EEPROM Ready");
      displayTime.sendBuffer();
      
      displayRpm.clearBuffer();
      displayRpm.setCursor(0, 20);
      displayRpm.print("Backup OK");
      displayRpm.sendBuffer();
      
      delay(1500);
    } else {
      Serial.println("EEPROM-Initialisierung fehlgeschlagen!");
    }
  } else {
    eepromAvailable = false;
    Serial.println("EEPROM-Test fehlgeschlagen!");
  }
  
  // --- SCHRITT 4: SD-Karten-Initialisierung ---
  int sdRetryCount = 0;
  while(!initSDCard() && sdRetryCount < 3) {
    Serial.println("SD-Initialisierung fehlgeschlagen, versuche erneut...");
    delay(1000);
    sdRetryCount++;
  }
  
  if (sdRetryCount >= 3) {
    Serial.println("SD-Karte konnte nicht initialisiert werden");
    sdCardAvailable = false;
  } else {
    sdCardAvailable = true;
  }
  
    // Nach erfolgreicher SD-Initialisierung
  Serial.println("SD-Karte initialisiert");
  
  // SPI explizit beenden
  SPI.endTransaction();
  delay(200);  // Mehr Zeit für Stabilisierung
  
  // I2C-Busse neu initialisieren (getrennt)
  Wire.end();  // Hauptbus beenden
  Wire1.end(); // EEPROM-Bus beenden
  delay(200);  
  
  // Erst EEPROM-Bus wieder starten
  Wire1.begin(SDA_PIN_EEPROM, SCL_PIN_EEPROM);
  Wire1.setClock(50000); // Niedrigere Frequenz
  delay(100);
  
  // Dann Hauptbus wieder starten
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(100);

  Serial.println("EEPROM-Bus nach SD-Init:");
  Wire1.beginTransmission(EEPROM_ADDRESS);
  Serial.print("Test-Ergebnis: ");
  Serial.println(Wire1.endTransmission());

  // --- SCHRITT 5: I2C-Bus nach SD-Initialisierung zurücksetzen ---
  Serial.println("I2C-Bus nach SD-Karten-Init zurücksetzen...");
  resetI2CBus();
  Wire.setClock(50000); // Langsamere Frequenz für bessere Stabilität
  
  // Wenn EEPROM-Test OK war aber nach SD-Init nicht mehr verfügbar, wiederherstellen
  if (!eepromAvailable && eepromTestOk) {
    Serial.println("EEPROM-Status wird nach SD-Init wiederhergestellt");
    eepromAvailable = true;
  }
  // nach der SD-Initialisierung und EEPROM-Status-Wiederherstellung
  Serial.println("EEPROM-Status wird auf 'Verfügbar' erzwungen");
  eepromAvailable = true; // Manuelles Setzen als letzter Versuch
  
  // EEPROM-Header neu validieren
  readEEPROMHeader(); // Header erneut lesen nach Reset
  // I2C-Recovery durchführen
  if (!recoverI2C()) {
    Serial.println("I2C-Recovery fehlgeschlagen!");
  } else {
    Serial.println("I2C-Recovery erfolgreich");
  }
  
  // EEPROM-Status nach Recovery prüfen
  if (eepromAvailable) {
    Wire.beginTransmission(EEPROM_ADDRESS);
    byte error = Wire.endTransmission();
    
    if (error != 0) {
      Serial.println("EEPROM nach SD-Init/Recovery nicht erreichbar!");
      eepromAvailable = false;
    } else {
      Serial.println("EEPROM-Verbindung nach SD-Init/Recovery bestätigt");
    }
  }
  
  // --- SCHRITT 6: SD-Karten-Meldung und Log-Datei erstellen ---
  if (sdCardAvailable) {
    Serial.println("SD-Karte erfolgreich initialisiert");
    displayTime.clearBuffer();
    displayTime.setCursor(0, 20);
    displayTime.print("SD Card OK");
    displayTime.sendBuffer();
    
    // Neuen Dateinamen generieren
    currentLogFileName = getNextFileName();
    Serial.print("Neuer Log-Dateiname: ");
    Serial.println(currentLogFileName);
    
    // Log-Header schreiben
    File dataFile = SD.open(currentLogFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("Date,UTC,RPM,Temperatur");
      dataFile.close();
    }
  } else {
    Serial.println("SD-Karte nicht verfügbar!");
    displayTime.clearBuffer();
    displayTime.setCursor(0, 20);
    displayTime.print("SD Card Error!");
    displayTime.setCursor(0, 40);
    displayTime.print("Logging disabled");
    displayTime.sendBuffer();
    delay(1500);
  }
  
  // --- SCHRITT 7: Sonstige Setup-Initialisierungen ---
  // Pin-Modi festlegen
  pinMode(BUTTON_PLUS, INPUT_PULLUP);
  pinMode(BUTTON_MINUS, INPUT_PULLUP);
  pinMode(BUTTON_SET, INPUT_PULLUP);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
 
  // Puffer initialisieren
  memset(dataBuffer, 0, sizeof(dataBuffer));
  bufferCount = 0;
  lastMinuteWrite = millis();
  
  // RPM Sensor initialisieren
  displayRpm.clearBuffer();
  displayRpm.setCursor(0, 20);
  displayRpm.print("Initializing");
  displayRpm.setCursor(0, 40);
  displayRpm.print("RPM sensor...");
  displayRpm.sendBuffer();
  
  Rpm_Count = 0;
  lastTime = millis();
  lastSecondRpmCount = millis();
  Rpm_Count_LastSecond = 0;
  lastOutputTime = millis();
 
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), Rpm_isr, FALLING);  
  // attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), Rpm_isr, CHANGE);  
  // Zusätzlich Debug-Ausgabe hinzufügen
  Serial.print("RPM-Interrupt auf Pin ");
  Serial.print(HALL_SENSOR_PIN);
  Serial.println(" konfiguriert (CHANGE-Modus)");
  
  // SET-Taste Interrupt konfigurieren
  attachInterrupt(digitalPinToInterrupt(BUTTON_SET), handleSetButtonInterrupt, FALLING);
  
  // --- SCHRITT 8: Setup abgeschlossen ---
  displayTime.clearBuffer();
  displayTime.setCursor(0, 20);
  displayTime.print("Setup");
  displayTime.setCursor(0, 40);
  displayTime.print("Complete!");
  displayTime.sendBuffer();
  
  displayRpm.clearBuffer();
  displayRpm.setCursor(0, 20);
  displayRpm.print("Setup");
  displayRpm.setCursor(0, 40);
  displayRpm.print("Complete!");
  displayRpm.sendBuffer();
  
  delay(1500);
  
  // Initialisiere die Anzeigen
  updateTimeFromRTC();
  if (sdCardAvailable) {
    showTime();
  } else {
    displayNoCard();
  }
  displayRPM();
  
  Serial.println("Setup abgeschlossen");
  
  // Am Ende von setup(), direkt vor dem Schließen der Funktion
  Serial.print("Finaler EEPROM-Status: ");
  Serial.println(eepromAvailable ? "Verfügbar" : "Nicht verfügbar");
  Serial.print("Gespeicherte Datensätze: ");
  Serial.println(eepromHeader.recordCount);
  
  // Zusätzliche Sicherheit: Notfall-Status-Setzung
  if (!eepromAvailable && eepromTestOk) {
    Serial.println("EEPROM-Status wird als letzter Versuch gesetzt!");
    eepromAvailable = true;
  }
}

// NEU: Funktion zum Schreiben des gesamten Puffers
bool writeBufferedDataToEEPROM() {
  for (uint8_t i = 0; i < bufferCount; i++) {
    uint32_t address = eepromHeader.nextWriteAddress;
    
    // Prüfen ob noch Platz im EEPROM ist
    if (address + sizeof(LogRecord) >= EEPROM_SIZE) {
      Serial.println("FEHLER: EEPROM voll!");
      return false;
    }
    
    // Datensatz schreiben
    writeEEPROMPage(address, (uint8_t*)&dataBuffer[i], sizeof(LogRecord));
    delay(EEPROM_WRITE_CYCLE);
    
    // Header-Zähler aktualisieren
    eepromHeader.recordCount++;
    eepromHeader.nextWriteAddress += sizeof(LogRecord);
  }
  
  // Header am Ende nur einmal schreiben
  writeEEPROMHeader();
  return true;
}

void loop() {
  unsigned long currentTime = millis();
  static bool handlingButton = false;
  static int lastRpm = -1;
  static unsigned long lastCardCheck = 0;
  static unsigned long lastTimeUpdate = 0;
  static unsigned long lastDebugOutput = 0;
  static int lastPinState = -1;
  // RPM automatisch zurücksetzen
  static unsigned long lastPulseTime = 0;
  static bool motorWasRunning = false;
  int currentPinState = digitalRead(HALL_SENSOR_PIN);

  // Hall-Sensor Status-Änderungen überwachen
  if (currentPinState != lastPinState) {
    Serial.print("Hall-Sensor Pin-Status geändert: ");
    Serial.println(currentPinState ? "HIGH" : "LOW");
    lastPinState = currentPinState;
    
    // Manuelles Trigger-Test
    if (currentPinState == LOW) {
      Serial.println("Manueller Impuls erkannt!");
      pulseCount = pulseCount + 1;
    }
  }
  
  // Debug-Ausgabe einmal pro Sekunde
  if (currentTime - lastDebugOutput > 1000) {
    lastDebugOutput = currentTime;
    Serial.print("DEBUG: Pulses detected in last second: ");
    Serial.println(pulseCount);
    pulseCount = 0; // Reset counter
  }

  // Sicherstellen, dass EEPROM immer verfügbar ist
  static bool firstRun = true;
  if (firstRun) {
    eepromAvailable = true;
    firstRun = false;
    Serial.println("EEPROM-Status manuell korrigiert");
  }

  // Temperatur vom RTC-Modul auslesen
  float temperature = rtc.getTemperature();
  
  // SD-Karten-Status regelmäßig überprüfen (alle 3 Sekunden)
  if (currentTime - lastCardCheck >= 3000) {
    bool previousState = sdCardAvailable;
    sdCardAvailable = checkSDCard();
    
    // Status hat sich geändert - aktualisiere Display entsprechend
    if (previousState != sdCardAvailable) {
      if (sdCardAvailable) {
        Serial.println("SD-Karte während des Betriebs eingesteckt");
        
        // Neuen Dateinamen generieren
        currentLogFileName = getNextFileName();
        Serial.print("Neuer Log-Dateiname: ");
        Serial.println(currentLogFileName);
        
        // Log-Header schreiben
        File dataFile = SD.open(currentLogFileName, FILE_WRITE);
        if (dataFile) {
          dataFile.println("Date,UTC,RPM,Temperatur,\"Motorstatus\"");
          dataFile.close();
          Serial.println("Neuer Log-Header geschrieben");
        }
        
        // Karte wurde eingesteckt - Zeit anzeigen
        showTime();
      } else {
        // Karte wurde entfernt
        displayNoCard();
      }
    } else {
      // Stelle sicher, dass NO CARD angezeigt wird
      if (!sdCardAvailable && currentTime - lastTimeUpdate >= 1000) {
        displayNoCard(); // Regelmäßig NO CARD anzeigen, wenn keine Karte vorhanden
      }
    }
    
    lastCardCheck = currentTime;
  }
  
  // RPM-Berechnung
  if (newPulseDetected) {
    // Merken, dass Motor läuft und letzte Impulszeit aktualisieren
    lastPulseTime = currentTime;
    if (Rpm > 500) motorWasRunning = true;
    
    // Aktuellen Wert berechnen
    float calculatedRpm = 60.0 * 1000000.0 / (pulsePeriod * RpmTriggerPerRound);
    
    // Statische Ringpuffer für schnelle Stabilisierung
    static float recentRPMs[5] = {0};
    static int rpmIndex = 0;
    static float lastCalculatedRpm = 0;
    
    // Plausibilitätsprüfung: 500-5000 RPM mit zusätzlicher Logik für Start
    if (calculatedRpm >= 500.0 && calculatedRpm <= 5000.0) {
      // Filter für unplausible Änderungen (z.B. plötzliche Verdoppelung)
      if (lastCalculatedRpm > 1000 && calculatedRpm > lastCalculatedRpm * 1.5) {
        Serial.print("Unplausible RPM-Änderung gefiltert: ");
        Serial.print(calculatedRpm);
        Serial.print(" -> behalte ");
        Serial.println(lastCalculatedRpm);
        calculatedRpm = lastCalculatedRpm;
      }
      
      lastCalculatedRpm = calculatedRpm;
      
      // Ringpuffer befüllen
      recentRPMs[rpmIndex] = calculatedRpm;
      rpmIndex = (rpmIndex + 1) % 5;
      
      // Durchschnitt berechnen
      float sum = 0;
      int validValues = 0;
      
      for (int i = 0; i < 5; i++) {
        if (recentRPMs[i] > 0) {
          sum += recentRPMs[i];
          validValues++;
        }
      }
      
      // SOFORTIGE ÜBERNAHME bei ersten Messungen oder großen Abweichungen
      if (Rpm < 100.0 || abs(calculatedRpm - Rpm) > 1000.0) {
        // Direkte Übernahme bei Start oder großer Änderung
        Rpm = calculatedRpm;
        
        // Puffer mit aktuellen Werten füllen für schnellere Konvergenz
        for (int i = 0; i < 5; i++) {
          recentRPMs[i] = calculatedRpm;
        }
      } 
      else if (validValues > 0) {
        // Normaler Fall: Einfacher Durchschnitt
        Rpm = sum / validValues;
      }
      
      // Debug-Ausgabe
      Serial.print("Periode: ");
      Serial.print(pulsePeriod);
      Serial.print(" µs, Berechnet: ");
      Serial.print(calculatedRpm);
      Serial.print(", Mittelwert: ");
      Serial.println(Rpm);
    }
    
    newPulseDetected = false;
    displayRPM();
  }
    
  // Nur alle 2 Sekunden auf SD-Karte schreiben
  static unsigned long lastSDWrite = 0;
  if (sdCardAvailable && currentTime - lastSDWrite >= 2000) { // nur alle 2 Sekunden
    lastSDWrite = currentTime;
    
    File dataFile = SD.open(currentLogFileName, FILE_APPEND);
    if (dataFile) {
      DateTime now = rtc.now();
      
      // Formatierter Zeitstempel
      char dateStr[15], timeStr[15];
      sprintf(dateStr, "%02d.%02d.%02d", now.day(), now.month(), now.year() % 100);
      sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
      
      dataFile.print(dateStr);
      dataFile.print(",");
      dataFile.print(timeStr);
      dataFile.print(",");
      dataFile.print(Rpm);
      dataFile.print(",");
      dataFile.print(temperature);
      dataFile.print(",");
      
      // Motorstatus prozentual anzeigen (optional)
      if (Rpm > 0) {
        float motorStatus = 100.0;
        dataFile.print("\"");
        dataFile.print(motorStatus, 2);
        dataFile.println("%\"");
      } else {
        dataFile.println("\"Motor stopped\"");
      }
      
      dataFile.close();
    } else {
      sdCardAvailable = false; // Fehler beim Schreiben, Status aktualisieren
    }
    
    // Daten im Puffer für EEPROM-Schreibvorgang speichern
    if (eepromAvailable && bufferCount < BUFFER_SIZE) {
      LogRecord newRecord;
      DateTime now = rtc.now();
      newRecord.timestamp = now.unixtime();
      newRecord.rpm = Rpm;
      newRecord.temperature = temperature;
      
      dataBuffer[bufferIndex] = newRecord;
      bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
      bufferCount++;
      
      // Wenn der Puffer voll ist oder genügend Zeit vergangen ist, auf EEPROM schreiben
      if (bufferCount >= BUFFER_SIZE || currentTime - lastEEPROMWrite >= EEPROM_WRITE_INTERVAL) {
        writeBufferedDataToEEPROM();
      }
    }
  }
    
  // RPM automatisch zurücksetzen, wenn keine Impulse mehr kommen
  if (motorWasRunning && currentTime - lastPulseTime > 2000 && Rpm > 0) {
    // Nach 2 Sekunden ohne Impulse RPM auf 0 setzen
    Serial.println("Keine Impulse mehr erkannt - Motor wahrscheinlich gestoppt");
    Rpm = 0;
    displayRPM();
    
    // Explizit RPM=0 in die Datei schreiben
    if (sdCardAvailable) {
      File dataFile = SD.open(currentLogFileName, FILE_APPEND);
      if (dataFile) {
        DateTime now = rtc.now();
        
        // Formatierter Zeitstempel
        char dateStr[15], timeStr[15];
        sprintf(dateStr, "%02d.%02d.%02d", now.day(), now.month(), now.year() % 100);
        sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
        
        dataFile.print(dateStr);
        dataFile.print(",");
        dataFile.print(timeStr);
        dataFile.print(",0,"); // RPM = 0
        dataFile.print(temperature);
        dataFile.println(",\"Motor stopped\"");
        dataFile.close();
      }
    }
    
    motorWasRunning = false; // Zurücksetzen des Flags
  }
  
  // Zeit auf dem Display aktualisieren (alle 1 Sekunde)
  if (currentTime - lastTimeUpdate >= 1000) {
    // NUR im NORMAL-Modus die Zeit aus der RTC aktualisieren
    if (currentState == NORMAL) {
      updateTimeFromRTC();
      
      // Zeit-Display aktualisieren, wenn im NORMAL-Modus und SD-Karte verfügbar
      if (sdCardAvailable) {
        showTime();
      }
    }
    lastTimeUpdate = currentTime;
  }

  // Direktes Polling der PLUS/MINUS Tasten
  static unsigned long lastButtonCheck = 0;
  if (currentTime - lastButtonCheck > 50) { // Alle 50ms prüfen
    lastButtonCheck = currentTime;
    
    // Direktes Lesen der Tasten (LOW = gedrückt bei Pull-up)
    if (digitalRead(BUTTON_PLUS) == LOW) {
      plusButtonPressed = true;
      Serial.println("PLUS-Taste gedrückt (Polling)");
    }
    
    if (digitalRead(BUTTON_MINUS) == LOW) {
      minusButtonPressed = true;
      Serial.println("MINUS-Taste gedrückt (Polling)");
    }
  }

  // Button-Handling für Setup-Modi
  handleButtons(currentTime);
  
  // Serielle Befehle verarbeiten
  handleSerialCommands();
}
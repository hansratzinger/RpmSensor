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
#define RpmTriggerPerRound 3 // 6 Impulse pro Umdrehung für präzisere Messung

#define IR_SENSOR_PIN 15 // GPIO15: Infrarotsensor Pin
#define LED_PIN 12      // GPIO12: Kontroll-LED Pin

// I2C Pins für einen Bus
#define SDA_PIN 21
#define SCL_PIN 22

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

// Struktur für den EEPROM-Header
struct EEPROMHeader {
  uint32_t recordCount;        // Anzahl der gespeicherten Datensätze
  uint32_t nextWriteAddress;   // Nächste freie Adresse zum Schreiben
  uint8_t initialized;         // Flag ob EEPROM initialisiert wurde (0xAA = ja)
};

// Struktur für einen Datensatz
struct LogRecord {
  uint32_t timestamp;          // UNIX-Timestamp
  uint16_t rpm;                // RPM-Wert
  int16_t temperature;         // Temperatur * a100 (um Dezimalstellen zu speichern)
};

// EEPROM-Verwaltungsvariablen
EEPROMHeader eepromHeader;
bool eepromAvailable = false;
uint32_t lastEepromWrite = 0;
const uint32_t EEPROM_WRITE_INTERVAL = 10000; // 10 Sekunden zwischen Schreibvorgängen

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
  if (interruptTime - lastInterruptTime > 5000) { // 5000 Mikrosekunden = 5 Millisekunden
    Rpm_Count = Rpm_Count + 1;
    Rpm_Count_LastSecond = Rpm_Count_LastSecond + 1;
    lastInterruptTime = interruptTime;
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

// Funktion zur Anzeige der Zeit (nur auf dem Zeit-Display)
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
  
  char dateStr[16]; 
  sprintf(dateStr, "%02d.%02d.%04d UTC", now.day(), now.month(), now.year());
  displayTime.print(dateStr);
  
  displayTime.sendBuffer();
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
  
  return true;
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
  
  // Beide Displays initialisieren
  displayTime.setI2CAddress(OLED_TIME_ADDR * 2); // U8G2 erwartet die Adresse im 8-Bit-Format
  displayTime.begin();
  displayTime.clearBuffer();
  
  displayRpm.setI2CAddress(OLED_RPM_ADDR * 2); // U8G2 erwartet die Adresse im 8-Bit-Format
  displayRpm.begin();
  displayRpm.clearBuffer();
  
  // Initialisierungsnachricht auf beiden Displays
  displayTime.setFont(u8g2_font_helvB12_tf);
  displayTime.setCursor(0, 20);
  displayTime.print("Time Display");
  displayTime.setCursor(0, 40);
  displayTime.print("Initialized");
  displayTime.sendBuffer();
  
  displayRpm.setFont(u8g2_font_helvB12_tf);
  displayRpm.setCursor(0, 20);
  displayRpm.print("RPM Display");
  displayRpm.setCursor(0, 40);
  displayRpm.print("Initialized");
  displayRpm.sendBuffer();
  
  delay(1000);
  
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
  
  delay(1000);
  
  // Mehrere Versuche für die SD-Karten-Initialisierung
  int sdRetryCount = 0;
  while(!initSDCard() && sdRetryCount < 3) {
    Serial.println("SD-Initialisierung fehlgeschlagen, versuche erneut...");
    delay(1000);
    sdRetryCount++;
  }
  
  if (sdRetryCount >= 3) {
    Serial.println("SD-Karte konnte nach mehreren Versuchen nicht initialisiert werden");
    sdCardAvailable = false;
  } else {
    sdCardAvailable = true;
  }

  // SD-Karte initialisieren
  displayTime.clearBuffer();
  displayTime.setCursor(0, 20);
  displayTime.print("Initializing");
  displayTime.setCursor(0, 40);
  displayTime.print("SD card...");
  displayTime.sendBuffer();
  
  sdCardAvailable = checkSDCard();
  
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
      
      // Bei der Anzeige des Dateinamens
      displayTime.clearBuffer();
      displayTime.setFont(u8g2_font_helvB10_tf);  // Etwas kleinere Schrift für mehr Platz
      displayTime.setCursor(0, 15);
      displayTime.print("Log file:");
      
      // Zeigt nur Datum und Uhrzeit an, ohne Pfad und Präfix
      String shortFileName = currentLogFileName;
      shortFileName.replace("/rpm_log_", "");  // Entferne Pfad und Präfix
      shortFileName.replace(".csv", "");       // Entferne Dateiendung
      
      // Aufteilen in zwei Zeilen
      int underscorePos = shortFileName.indexOf('_');
      if (underscorePos != -1) {
        // Datum
        displayTime.setCursor(0, 35);
        displayTime.print(shortFileName.substring(0, underscorePos));
        
        // Zeit
        displayTime.setCursor(0, 55);
        displayTime.print(shortFileName.substring(underscorePos + 1));
      } else {
        // Falls kein Unterstrich vorhanden ist, zeige den gesamten Namen
        displayTime.setCursor(0, 35);
        displayTime.print(shortFileName);
      }
      
      displayTime.sendBuffer();
    }
  } else {
    Serial.println("SD-Karte konnte nicht initialisiert werden!");
    
    displayTime.clearBuffer();
    displayTime.setCursor(0, 20);
    displayTime.print("SD Card Error!");
    displayTime.setCursor(0, 40);
    displayTime.print("Logging disabled");
    displayTime.sendBuffer();
  }
  
  delay(1000);
  
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
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), Rpm_isr, CHANGE);
  
  // SET-Taste Interrupt konfigurieren
  attachInterrupt(digitalPinToInterrupt(BUTTON_SET), handleSetButtonInterrupt, FALLING);
  
  // Setup abgeschlossen
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
  showTime();
  displayRPM();
  
  Serial.println("Setup abgeschlossen");
}

void loop() {
  unsigned long currentTime = millis();
  static bool handlingButton = false;
  static int lastRpm = -1;
  static unsigned long lastCardCheck = 0;
  static unsigned long lastTimeUpdate = 0;
  
  // Temperatur vom RTC-Modul auslesen
  float temperature = rtc.getTemperature();
  
  // SD-Karten-Status regelmäßig überprüfen (alle 3 Sekunden)
  if (currentTime - lastCardCheck >= 3000) {
    bool previousState = sdCardAvailable;
    sdCardAvailable = checkSDCard();
    
    // Status hat sich geändert - aktualisiere Display entsprechend
    if (previousState != sdCardAvailable) {
      if (sdCardAvailable) {
        // Karte wurde eingesteckt
        showTime();
      } else {
        // Karte wurde entfernt
        displayNoCard();
      }
    }
    
    lastCardCheck = currentTime;
  }
  
  // RPM berechnen (einmal pro Sekunde)
  if (currentTime - lastSecondRpmCount >= 1000) {
    // Speichere die Anzahl der Impulse VOR dem Zurücksetzen
    unsigned long capturedImpulses = Rpm_Count_LastSecond;
    
    // Berechnen mit dem korrigierten Wert für RpmTriggerPerRound
    Rpm = capturedImpulses * 60 / RpmTriggerPerRound;
    
    // Debug-Ausgabe
    Serial.print("Erfasste Impulse in der letzten Sekunde: ");
    Serial.println(capturedImpulses);
    Serial.print("RPM: ");
    Serial.println(Rpm);
    
    // Jetzt erst zurücksetzen
    Rpm_Count_LastSecond = 0;
    Rpm_Count = 0;
    lastSecondRpmCount = currentTime;
    
    // Aktualisiere RPM-Display
    displayRPM();
    
    // Logge die Daten auf die SD-Karte nur wenn sie verfügbar ist
    if (sdCardAvailable) {
      File dataFile = SD.open(currentLogFileName, FILE_APPEND);
      if (dataFile) {
        DateTime now = rtc.now();
        
        // Datums- und Zeitformat mit führenden Nullen: YYYY-MM-DD,HH:MM:SS
        dataFile.print(now.year()); dataFile.print("-");
        // Führende Nullen für Monat
        if (now.month() < 10) dataFile.print("0");
        dataFile.print(now.month()); dataFile.print("-");
        // Führende Nullen für Tag
        if (now.day() < 10) dataFile.print("0");
        dataFile.print(now.day()); dataFile.print(",");
        
        // Führende Nullen für Stunde
        if (now.hour() < 10) dataFile.print("0");
        dataFile.print(now.hour()); dataFile.print(":");
        // Führende Nullen für Minute
        if (now.minute() < 10) dataFile.print("0");
        dataFile.print(now.minute()); dataFile.print(":");
        // Führende Nullen für Sekunde
        if (now.second() < 10) dataFile.print("0");
        dataFile.print(now.second()); dataFile.print(",");
        
        dataFile.print(Rpm); dataFile.print(",");
        dataFile.println(temperature);
        
        dataFile.close();
      } else {
        sdCardAvailable = false; // Fehler beim Schreiben, Status aktualisieren
      }
    }
  }
  
  // Zeit-Display alle Sekunde aktualisieren
  if (currentTime - lastTimeUpdate >= 1000) {
    if (sdCardAvailable) {
      updateTimeFromRTC();
      showTime();
    } else {
      displayNoCard();
    }
    lastTimeUpdate = currentTime;
  }
  
  // Überprüfen, ob die SET-Taste gedrückt wurde
  if (buttonPressed && !handlingButton) {
    handlingButton = true; // Flag setzen
    buttonPressed = false;
    delay(200); // Entprellung
    setupRTCWithButtons();
    
    // Nach dem Setup beide Displays neu zeichnen
    updateTimeFromRTC();
    showTime();
    displayRPM();
    
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
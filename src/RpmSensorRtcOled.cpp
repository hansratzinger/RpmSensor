// -------------------------------------------------
// RpmSensorRtcOled.cpp
// -------------------------------------------------
// Usage for measuring RMP (Revolutions per Minute) with an infrared sensor
// and displaying the result on OLED displays. The DS3231 RTC is used to keep track of time.
// The SD card is used to log the data. The program allows setting the RTC time using buttons on the ESP32 board.
// ESP32 DS3231 RTC + Two OLED Displays + Infrared Sensor
// RELEASE 4.0 HR 2025-04-30 NK
// -------------------------------------------------
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

// RPM-Messung Konfiguration
#define RpmTriggerPerRound 2 // Impulse pro Umdrehung
#define IR_SENSOR_PIN 15     // GPIO15: Infrarotsensor Pin
#define LED_PIN 12          // GPIO12: Kontroll-LED Pin

// I2C Pins für einen gemeinsamen Bus
#define SDA_PIN 21
#define SCL_PIN 22

// OLED Display Adressen (7-Bit Format)
#define OLED1_ADDR 0x3D // 0x7A in 8-Bit-Format (RPM-Display)
#define OLED2_ADDR 0x3C // 0x78 in 8-Bit-Format (Zeit-Display)
#define CLOCK_INTERRUPT_PIN 14

// Initialisierung der OLED-Displays am gleichen Bus
U8G2_SSD1306_128X64_NONAME_F_HW_I2C displayRPM(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C displayTime(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Globale Statusvariable für SD-Karte
bool sdCardAvailable = false;

// Globale Variable für den aktuellen Dateinamen
String currentLogFileName;

// RTC Modul
RTC_DS3231 rtc;

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
int day = 1;     
int month = 5;   
int year = 2025; 

// RPM-Messung
volatile unsigned long pulsesSinceLastUpdate = 0;
int Rpm = 0;
unsigned long lastTime;           
unsigned long lastOutputTime = 0; 
unsigned long lastSecondRpmCount = 0;

// Entprellzeiten für Interrupts
const unsigned long DEBOUNCE_TIME = 200;       // 0.2ms für RPM-Sensor
const unsigned long BUTTON_DEBOUNCE_TIME = 500; // 500ms für Buttons
volatile unsigned long lastRpmInterruptTime = 0;
volatile unsigned long lastButtonInterruptTime = 0;

// Interrupt-Handler
void IRAM_ATTR Rpm_isr() {
  unsigned long interruptTime = micros();
  
  if (interruptTime - lastRpmInterruptTime > DEBOUNCE_TIME) {
    pulsesSinceLastUpdate++;
    lastRpmInterruptTime = interruptTime;
    digitalWrite(LED_PIN, HIGH);
  }
}

void IRAM_ATTR handleSetButtonInterrupt() {
  unsigned long interruptTime = millis();
  
  if (interruptTime - lastButtonInterruptTime > BUTTON_DEBOUNCE_TIME) {
    buttonPressed = true;
    lastButtonInterruptTime = interruptTime;
  }
}

// Funktion zur Aktualisierung der Zeit vom RTC
void updateTimeFromRTC() {
  DateTime now = rtc.now();
  hour = now.hour();
  minute = now.minute();
  second = now.second();
}

// Funktion zur Anzeige der Zeit auf dem Zeit-Display
void showTime() {
  displayTime.clearBuffer();
  
  DateTime now = rtc.now();
  
  displayTime.setFont(u8g2_font_inb19_mf);
  displayTime.setCursor(5, 30);
  
  char timeStr[9];
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  displayTime.print(timeStr);
  
  displayTime.setFont(u8g2_font_helvB10_tf);
  displayTime.setCursor(15, 55);
  
  char dateStr[16];
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
  
  displayRPM.setFont(u8g2_font_inb24_mf);
  displayRPM.setCursor(5, 60);
  displayRPM.print(Rpm);
  
  // Wenn keine SD-Karte verfügbar ist, "NO CARD" anzeigen
  if (!sdCardAvailable) {
    displayRPM.setFont(u8g2_font_helvB08_tf);
    displayRPM.setCursor(80, 20);
    displayRPM.print("NO");
    displayRPM.setCursor(80, 30);
    displayRPM.print("CARD");
  }
  
  displayRPM.sendBuffer();
}

// Funktion zur Anzeige der "NO CARD" Warnung auf dem Zeit-Display
void showNoCard() {
  displayTime.clearBuffer();
  displayTime.setFont(u8g2_font_inb24_mf);
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
      if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
        return 29;
      } else {
        return 28;
      }
    case 4: case 6: case 9: case 11: 
      return 30;
    default:
      return 31;
  }
}

// Funktion zur Anzeige des Setup-Bildschirms
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
      
      // Warte bis die SET-Taste losgelassen wird
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
  // Stelle sicher, dass alle Tasten losgelassen wurden
  while(digitalRead(BUTTON_SET) == LOW || 
        digitalRead(BUTTON_PLUS) == LOW || 
        digitalRead(BUTTON_MINUS) == LOW) {
    delay(10);
  }
  
  delay(200);
  
  // Interrupt wieder aktivieren
  attachInterrupt(digitalPinToInterrupt(BUTTON_SET), handleSetButtonInterrupt, FALLING);
}

// Funktion zum Generieren eines Dateinamens mit aktuellem Datum und Uhrzeit
String getNextFileName() {
  DateTime now = rtc.now();
  
  char fileName[50];
  sprintf(fileName, "/rpm_log_%04d-%02d-%02d_%02d-%02d-%02d.csv", 
          now.year(), now.month(), now.day(), 
          now.hour(), now.minute(), now.second());
  
  return String(fileName);
}

// Funktion zur Überprüfung der SD-Karte während der Laufzeit
bool checkSDCard() {
  if (!SD.begin(SD_CS)) {
    return false;
  }
  
  // Testen, ob wir lesen/schreiben können
  File testFile = SD.open("/rpm_log_test.txt", FILE_WRITE);
  if (testFile) {
    testFile.close();
    return true;
  }
  
  return false;
}

void setup() {
  Serial.begin(115200);
  
  // I2C-Bus initialisieren
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Pin-Modi festlegen
  pinMode(BUTTON_PLUS, INPUT_PULLUP);
  pinMode(BUTTON_MINUS, INPUT_PULLUP);
  pinMode(BUTTON_SET, INPUT_PULLUP);
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Displays initialisieren
  displayRPM.begin();
  displayRPM.setI2CAddress(0x7A); // 8-bit
  displayRPM.clearBuffer();
  displayRPM.setFont(u8g2_font_helvB12_tf);
  displayRPM.setCursor(0, 30);
  displayRPM.print("RPM Sensor");
  displayRPM.setCursor(0, 50);
  displayRPM.print("Initializing...");
  displayRPM.sendBuffer();
  
  displayTime.begin();
  displayTime.setI2CAddress(0x78); // 8-bit
  
  // RTC initialisieren
  if (!rtc.begin()) {
    displayTime.clearBuffer();
    displayTime.setFont(u8g2_font_helvB12_tf);
    displayTime.setCursor(0, 30);
    displayTime.print("RTC Error!");
    displayTime.sendBuffer();
    while (1) delay(10);
  }
  
  // RTC Konfiguration
  rtc.disable32K();
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);
  rtc.disableAlarm(2);
  
  // Zeit aus RTC holen oder bei Bedarf einstellen
  if (rtc.lostPower()) {
    // Standardzeit setzen (30.04.2025 12:00:00)
    rtc.adjust(DateTime(2025, 4, 30, 12, 0, 0));
  }
  
  updateTimeFromRTC();
  
  // SD-Karte initialisieren
  sdCardAvailable = checkSDCard();
  
  if (sdCardAvailable) {
    // Neuen Dateinamen generieren
    currentLogFileName = getNextFileName();
    
    // Log-Header schreiben
    File dataFile = SD.open(currentLogFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("Date,UTC,RPM,Temperatur");
      dataFile.close();
    }
  }
  
  // RPM Sensor initialisieren
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), Rpm_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_SET), handleSetButtonInterrupt, FALLING);
  
  // Initialisiere die Anzeigen
  showTime();
  showRPM();
}

void loop() {
  unsigned long currentTime = millis();
  
  // LED nach 5ms ausschalten für sichtbares Blinken
  static unsigned long ledTurnOffTime = 0;
  if (digitalRead(LED_PIN) == HIGH && (currentTime - ledTurnOffTime > 5)) {
    digitalWrite(LED_PIN, LOW);
  } else if (digitalRead(LED_PIN) == HIGH) {
    ledTurnOffTime = currentTime;
  }
  
  // Button-Handler
  static bool handlingButton = false;
  if (buttonPressed && !handlingButton) {
    handlingButton = true;
    buttonPressed = false;
    setupRTCWithButtons();
    handlingButton = false;
  }
  
  // RPM-Berechnung
  if (currentTime - lastSecondRpmCount >= 1000) {
    // Direkte Berechnung
    Rpm = pulsesSinceLastUpdate * 60 / RpmTriggerPerRound;
    
    // Zähler zurücksetzen
    pulsesSinceLastUpdate = 0;
    lastSecondRpmCount = currentTime;
  }

  // Zeit und RPM Anzeige aktualisieren (5x pro Sekunde)
  if (currentTime - lastOutputTime > 200) {
    showTime();
    showRPM();
    lastOutputTime = currentTime;
  }
  
  // SD-Karten-Logging (alle 5 Sekunden)
  static unsigned long lastLogTime = 0;
  if (currentTime - lastLogTime >= 5000 && sdCardAvailable) {
    // Aktuelle Zeit und Temperatur
    DateTime now = rtc.now();
    float temperature = rtc.getTemperature();
    
    // Log-Datei öffnen und Daten schreiben
    File dataFile = SD.open(currentLogFileName, FILE_WRITE);
    if (dataFile) {
      // Datum im Format YYYY-MM-DD
      char dateStr[11];
      sprintf(dateStr, "%04d-%02d-%02d", now.year(), now.month(), now.day());
      
      // Zeit im Format HH:MM:SS
      char timeStr[9];
      sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
      
      // Daten in CSV-Format schreiben
      dataFile.print(dateStr);
      dataFile.print(",");
      dataFile.print(timeStr);
      dataFile.print(",");
      dataFile.print(Rpm);
      dataFile.print(",");
      dataFile.println(temperature);
      
      dataFile.close();
    }
    
    lastLogTime = currentTime;
  }
  
  // Überprüfung der SD-Karte alle 30 Sekunden
  static unsigned long lastSDCheck = 0;
  if (currentTime - lastSDCheck >= 30000) {
    bool sdStatus = checkSDCard();
    
    // Wenn sich der SD-Kartenstatus geändert hat
    if (sdStatus != sdCardAvailable) {
      sdCardAvailable = sdStatus;
      
      if (sdCardAvailable) {
        // SD-Karte wurde eingesteckt
        currentLogFileName = getNextFileName();
        
        // Log-Header schreiben
        File dataFile = SD.open(currentLogFileName, FILE_WRITE);
        if (dataFile) {
          dataFile.println("Date,UTC,RPM,Temperatur");
          dataFile.close();
        }
      }
      
      // Display aktualisieren wegen SD-Status-Änderung
      showRPM();
    }
    
    lastSDCheck = currentTime;
  }
}
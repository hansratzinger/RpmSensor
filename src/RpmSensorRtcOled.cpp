// Project: RpmSensorRtcOled
// HR 2025-04-10 00:45 NK


/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete instructions at https://RandomNerdTutorials.com/esp32-ds3231-real-time-clock-arduino/
*********/

// based on the RTClib: implementation of an alarm using DS3231 https://github.com/adafruit/RTClib

// OLED display library
#include <Arduino.h>
#include "TFT_eSPI.h"
#include "User_Setup.h" // TFT_eSPI library setup file
#include <FS.h>
#include <SD.h>
#include <SPI.h>


#define TRIGGERS_PER_REV 1 // Anzahl der Impulse pro Umdrehung

#define IR_SENSOR_PIN    15 // GPIO15: Infrarotsensor Pin
#define LED_PIN           2 // GPIO2: Kontroll-LED Pin
#define SD_CS 13    // GPIO Chip Select für die SD-Karte

TFT_eSPI tft = TFT_eSPI();

// based on the RTClib: implementation of an alarm using DS3231 https://github.com/adafruit/RTClib
// DS3231 RTC library
#include <RTClib.h>
RTC_DS3231 rtc;

bool TEST = true; // set to true to enable debug output

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 14

volatile unsigned long Rpm_Count; // Zähler für Interrupts
int Rpm;                        // variable to store the RPM value          
unsigned long lastTime;           // Variable für die letzte Zeitmessung
volatile unsigned long lastInterruptTime = 0;  // Zeit des letzten Interrupts

unsigned long lastOutputTime = 0; // Zeitpunkt der letzten Ausgabe
unsigned long lastSecondRpmCount = 0;
unsigned long Rpm_Count_LastSecond = 0;
float boardTime = 0;
const int measurementTime = 1; // Messzeit in Sekunden

void IRAM_ATTR Rpm_isr() {
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 1) { // Entprellzeit von 1ms
    Rpm_Count++;
    lastInterruptTime = interruptTime;
  }
}

// LED Funktion vereinfacht
void setLed(bool state, uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, state);
}

void debugPinSettings() {
    Serial.println("Pin Settings:");
    Serial.print("TFT_CS: ");
    Serial.println(TFT_CS);
    Serial.print("SD_CS: ");
    Serial.println(SD_CS);
    Serial.print("TFT_MISO: ");
    Serial.println(TFT_MISO);
    Serial.print("TFT_MOSI: ");
    Serial.println(TFT_MOSI);
    Serial.print("TFT_SCLK: ");
    Serial.println(TFT_SCLK);
    Serial.print("TFT_DC: ");
    Serial.println(TFT_DC);
    Serial.print("TFT_RST: ");
    Serial.println(TFT_RST);      
    Serial.print("IR_SENSOR_PIN: ");
    Serial.println(IR_SENSOR_PIN);
    Serial.print("LED_PIN: ");
    Serial.println(LED_PIN);
}

void selectDisplay() {
    Serial.println("Display ausgewählt");
    Serial.println("deaktivieren SD_CS: " + String(TFT_CS));
    digitalWrite(SD_CS, HIGH);  // SD-Karte deaktivieren
    Serial.println("aktivieren TFT_CS: " + String(SD_CS));  
    digitalWrite(TFT_CS, LOW);  // Display aktivieren
}

void selectSDCard() {
    Serial.println("SD-Karte ausgewählt");
    Serial.println("deaktivieren TFT_CS: " + String(SD_CS));
    digitalWrite(TFT_CS, HIGH); // Display deaktivieren
    Serial.println("aktivieren SD_CS: " + String(TFT_CS));
    digitalWrite(SD_CS, LOW);   // SD-Karte aktivieren
}

void testOLED() {
  TFT_eSPI tft = TFT_eSPI();  
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("Display OK");  
  delay(3000);
  tft.fillScreen(TFT_BLACK);
  delay(2000);
}

String currentTime(){
  // Get the current time from the RTC
  String currentTime ="";
  if (rtc.now().hour() < 10) currentTime = "0"; // Add leading zero for hour 
  currentTime += String(rtc.now().hour(), DEC) + ":"; 
  if (rtc.now().minute() < 10) currentTime += "0"; // Add leading zero for minute  
  currentTime += String(rtc.now().minute(), DEC) + ":"; // Add minute with leading zero  
  if (rtc.now().second() < 10) currentTime += "0"; // Add leading zero for second 
  currentTime += String(rtc.now().second(), DEC); // Add second with leading zero
  return currentTime;
}

String currentDate(){
  // Get the current time from the RTC
  String currentDate;
  currentDate = String(rtc.now().year(), DEC) + "-";
  if (rtc.now().month() < 10) currentDate += "0"; // Add leading zero for month
  currentDate += String(rtc.now().month(), DEC) + "-"; // Add month with leading zero
  if (rtc.now().day() < 10) currentDate += "0"; // Add leading zero for day 
  currentDate += String(rtc.now().day(), DEC); // Add day with leading zero);
  return currentDate;
}

void displayRpm(int rpm) {
    tft.fillScreen(TFT_BLACK);  // clear screen
    tft.setCursor(10, 20);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(5);
    tft.setTextWrap(true);
    tft.print(rpm);
}

void displayTimestamp() {
    // tft.fillScreen(ST77XX_BLACK); // clear screen
    tft.setCursor(10, 75);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(3);
    // tft.setTextWrap(true);
    tft.print(currentTime());
    tft.setTextSize(2);
    tft.setCursor(25, 110);
    tft.print(currentDate());
}

void monitorDebug() {
    Serial.print(currentTime());
    Serial.print(" ");  
    Serial.print(currentDate());
    Serial.print(" ");
    Serial.print("Rpm_Count: ");
    Serial.println(Rpm_Count);
    Serial.print("Rpm: ");
    Serial.println(Rpm);
}

File openLogFile() {
  String fileName = "/" + currentDate() + ".csv"; // Dateiname basierend auf dem aktuellen Datum
  bool fileExists = SD.exists(fileName);         // Prüfen, ob die Datei bereits existiert
  File file = SD.open(fileName, FILE_APPEND);    // Datei im Anhänge-Modus öffnen
  if (!file) {
      Serial.println("Fehler beim Öffnen der Datei: " + fileName);
  } else {
      Serial.println("Datei geöffnet: " + fileName);
      if (!fileExists) {
          // Schreibe Spaltenüberschriften, wenn die Datei neu erstellt wurde
          file.println("date,time,rpm");
          Serial.println("Spaltenüberschriften hinzugefügt.");
      }
  }
  return file;
}

void logData(File file, int rpm) {
  if (file) {
      String logEntry = currentDate() + "," + currentTime() + "," + String(rpm);
      file.println(logEntry); // Daten in die Datei schreiben
      Serial.print("Daten geschrieben: ");
      Serial.println(logEntry);
  } else {
      Serial.println("Datei ist nicht geöffnet!");
  }
}

void closeLogFile(File file) {
  if (file) {
      file.close();
      Serial.println("Datei geschlossen.");
  }
}

void displaySDCardWarning() {
  tft.fillScreen(TFT_RED);  
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 20);
  tft.print("No SD-card!");
  delay(2500);
  tft.setCursor(10, 60);
  tft.print("Restarting");
  tft.setCursor(10, 80);
  tft.print("in 5 sec...");
  delay(5000);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 40);
  tft.print("Restarting");
  delay(1000);
  tft.fillScreen(TFT_BLACK);
}


void setup() {
    Serial.begin(115200);
    Serial.println("Setup gestartet");
    delay(2000); // Warten auf den Serial Monitor
    debugPinSettings(); // Pin-Einstellungen debuggen
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // RTC initialisieren
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC!");
        Serial.flush();
        while (1) delay(10);
    }
    rtc.disable32K();
    pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);

    rtc.clearAlarm(1);
    rtc.clearAlarm(2);
    rtc.writeSqwPinMode(DS3231_OFF);
    rtc.disableAlarm(2);

    // SPI wird in der library initialisiert
    // CS-Pins als Ausgang setzen
    pinMode(TFT_CS, OUTPUT);
    pinMode(SD_CS, OUTPUT);

    // Beide Geräte deaktivieren
    digitalWrite(TFT_CS, HIGH);
    digitalWrite(SD_CS, HIGH);

    // Display initialisieren
    Serial.println("Versuche, Display zu initialisieren...");
    selectDisplay();
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.print("Display OK");
    digitalWrite(TFT_CS, HIGH); // Display deaktivieren
    delay(3000); // Warten auf den Serial Monitor
    // SD-Karte initialisieren
    Serial.println("Versuche, SD-Karte zu initialisieren...");
    selectSDCard();
    // SD-Karten initialisieren
    while (!SD.begin(SD_CS, tft.getSPIinstance())) {
        Serial.println("SD-Karte konnte nicht initialisiert werden!");
        delay(1000); // Warten auf den Serial Monitor
        selectDisplay();
        displaySDCardWarning();
        selectSDCard;
      }

    digitalWrite(SD_CS, HIGH); // SD-Karte deaktivieren

    //RPM sensor initialization
    setLed(true, LED_PIN);    
    pinMode(IR_SENSOR_PIN, INPUT_PULLUP); // Setze den IR-Sensor-Pin als Eingang mit Pull-Up Widerstand
    pinMode(LED_PIN, OUTPUT);          
  
    Serial.println("Rpm Sensor gestartet");
  
    Rpm_Count = 0;          // Initialisiere den Zähler
    lastTime = millis();    // Initialisiere die letzte Zeitmessung
    lastSecondRpmCount = millis(); // Initialisiere lastSecondRpmCount
    Rpm_Count_LastSecond = 0; // Initialisiere Rpm_Count_LastSecond
    lastOutputTime = millis();
  
    attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), Rpm_isr, FALLING); // Interrupt bei fallender Flanke
    setLed(false, LED_PIN);
    Serial.println("Setup abgeschlossen");
}


void loop() {
    unsigned long currentTime = millis();
    if (currentTime - lastOutputTime >= measurementTime * 1000) {
        unsigned long impulseCount = Rpm_Count - Rpm_Count_LastSecond;
        Rpm = ((int)impulseCount * 60) / measurementTime / TRIGGERS_PER_REV;

        lastOutputTime = currentTime;
        Rpm_Count_LastSecond = Rpm_Count;

        if (TEST) {
          monitorDebug();
        } 

        // Display aktivieren und Daten anzeigen
        selectDisplay();
        displayRpm(Rpm);
        displayTimestamp();
        
        // SD-Karte aktivieren und Daten speichern
        selectSDCard();
        File logFile = openLogFile(); // Datei öffnen oder erstellen
        logData(logFile, Rpm);        // Daten schreiben
        closeLogFile(logFile);       // Datei schließen
        
    }
}

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

#define BUTTON_PLUS 32  // GPIO für die "+"-Taste
#define BUTTON_MINUS 33 // GPIO für die "-"-Taste
#define BUTTON_SET 27   // GPIO für die "SET"-Taste

#define TRIGGERS_PER_REV 1 // Anzahl der Impulse pro Umdrehung

#define IR_SENSOR_PIN 15 // GPIO15: Infrarotsensor Pin
#define LED_PIN 2        // GPIO2: Kontroll-LED Pin
#define SD_CS 13         // GPIO Chip Select für die SD-Karte

TFT_eSPI tft = TFT_eSPI();

// based on the RTClib: implementation of an alarm using DS3231 https://github.com/adafruit/RTClib
// DS3231 RTC library
#include <RTClib.h>
RTC_DS3231 rtc;

bool TEST = true; // set to true to enable debug output

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 14

volatile unsigned long Rpm_Count; // Zähler für Interrupts
int Rpm;                          // variable to store the RPM value
unsigned long lastTime;           // Variable für die letzte Zeitmessung

unsigned long lastOutputTime = 0; // Zeitpunkt der letzten Ausgabe
unsigned long lastSecondRpmCount = 0;
unsigned long Rpm_Count_LastSecond = 0;
unsigned long lastSetButtonPressTime = 0; // Zeit des letzten "SET"-Tastendrucks
float boardTime = 0;
const int measurementTime = 1; // Messzeit in Sekunden

volatile bool startRTCSetup = false;          // Flag, um die Zeiteinstellung zu starten
volatile unsigned long lastInterruptTime = 0; // Zeit des letzten Interrupts
void IRAM_ATTR handleSetButtonInterrupt()
{
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 500)
  {                       // 500 ms Entprellzeit
    startRTCSetup = true; // Setze das Flag, wenn der Interrupt ausgelöst wird
    lastInterruptTime = interruptTime;
  }
}

void IRAM_ATTR Rpm_isr()
{
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 1)
  { // Entprellzeit von 1ms
    Rpm_Count++;
    lastInterruptTime = interruptTime;
  }
}

String readGPIOStatus(int pin)
{
  int pinStatus = digitalRead(pin); // Status des GPIO-Pins auslesen
  if (pinStatus == HIGH)
  {
    return "HIGH";
  }
  else
  {
    return "LOW";
  }
}

// LED Funktion vereinfacht
void setLed(bool state, uint8_t pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, state);
}

void debugButtonSettings()
{
  Serial.println("Button Settings:");
  Serial.print("BUTTON_PLUS: ");
  Serial.println(readGPIOStatus(BUTTON_PLUS));
  Serial.print("BUTTON_MINUS: ");
  Serial.println(readGPIOStatus(BUTTON_MINUS));
  Serial.print("BUTTON_SET: ");
  Serial.println(readGPIOStatus(BUTTON_SET));
}

void setupRTCWithButtons()
{
  enum State
  {
    SET_HOUR,
    SET_MINUTE,
    SET_SECOND,
    COMPLETE
  }; // Zustände für die Zeiteinstellung
  State currentState = SET_HOUR;

  unsigned long lastButtonPressTime = 0;   // Zeit des letzten Tastendrucks
  const unsigned long debounceDelay = 200; // Entprellzeit in Millisekunden

  int hour = rtc.now().hour();     // Aktuelle Stunde von der RTC
  int minute = rtc.now().minute(); // Aktuelle Minute von der RTC
  int second = rtc.now().second(); // Aktuelle Sekunde von der RTC

  bool stateChanged = true; // Flag, um anzuzeigen, ob sich der Zustand geändert hat

  // Warten, bis der BUTTON_SET losgelassen wird
  while (digitalRead(BUTTON_SET) == LOW)
  {
    delay(10); // Kurze Verzögerung, um den Button-Status zu überprüfen
  }

  while (currentState != COMPLETE)
  {
    if (stateChanged)
    {
      // Bildschirm nur aktualisieren, wenn sich der Zustand geändert hat
      tft.fillScreen(TFT_BLACK);
      tft.setTextSize(2);
      tft.setCursor(10, 14);
      tft.printf("Setting Time");

      if (currentState == SET_HOUR)
      {
        tft.setCursor(10, 38);
        tft.printf("UTC Hour: %02d", hour);
      }
      else if (currentState == SET_MINUTE)
      {
        tft.setCursor(10, 38);
        tft.printf("Minute:   %02d", minute);
      }
      else if (currentState == SET_SECOND)
      {
        tft.setCursor(10, 38);
        tft.printf("Second:   %02d", second);
      }

      tft.setTextSize(1);
      tft.setCursor(45, 110);
      tft.printf("+ / - / Set");

      stateChanged = false; // Anzeige wurde aktualisiert
    }

    // Tastenabfragen mit Entprellung
    unsigned long currentTime = millis();

    // "+"-Taste gedrückt
    if (digitalRead(BUTTON_PLUS) == LOW && (currentTime - lastButtonPressTime > debounceDelay))
    {
      lastButtonPressTime = currentTime;
      if (currentState == SET_HOUR)
      {
        hour = (hour + 1) % 24; // Stunden erhöhen (0-23)
      }
      else if (currentState == SET_MINUTE)
      {
        minute = (minute + 1) % 60; // Minuten erhöhen (0-59)
      }
      else if (currentState == SET_SECOND)
      {
        second = (second + 1) % 60; // Sekunden erhöhen (0-59)
      }
      stateChanged = true; // Zustand geändert, Anzeige aktualisieren
    }

    // "-"-Taste gedrückt
    if (digitalRead(BUTTON_MINUS) == LOW && (currentTime - lastButtonPressTime > debounceDelay))
    {
      lastButtonPressTime = currentTime;
      if (currentState == SET_HOUR)
      {
        hour = (hour - 1 + 24) % 24; // Stunden verringern (0-23)
      }
      else if (currentState == SET_MINUTE)
      {
        minute = (minute - 1 + 60) % 60; // Minuten verringern (0-59)
      }
      else if (currentState == SET_SECOND)
      {
        second = (second - 1 + 60) % 60; // Sekunden verringern (0-59)
      }
      stateChanged = true; // Zustand geändert, Anzeige aktualisieren
    }

    // "SET"-Taste gedrückt
    if (digitalRead(BUTTON_SET) == LOW && (currentTime - lastButtonPressTime > debounceDelay))
    {
      lastButtonPressTime = currentTime;

      // Warten, bis der Button losgelassen wird
      while (digitalRead(BUTTON_SET) == LOW)
      {
        delay(10); // Kurze Verzögerung, um den Button-Status zu überprüfen
      }

      if (currentState == SET_HOUR)
      {
        currentState = SET_MINUTE; // Wechsel zur Minuten-Einstellung
      }
      else if (currentState == SET_MINUTE)
      {
        currentState = SET_SECOND; // Wechsel zur Sekunden-Einstellung
      }
      else if (currentState == SET_SECOND)
      {
        currentState = COMPLETE; // Zeiteinstellung abschließen
      }
      stateChanged = true; // Zustand geändert, Anzeige aktualisieren
    }

    delay(50); // Kurze Verzögerung, um die Anzeige zu stabilisieren
  }

  // RTC mit den neuen Werten aktualisieren
  rtc.adjust(DateTime(rtc.now().year(), rtc.now().month(), rtc.now().day(), hour, minute, second));
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 10);
  tft.print("UTC time set");
  delay(200);
}

void debugPinSettings()
{
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
  Serial.print("BUTTON_PLUS: ");
  Serial.println(BUTTON_PLUS);
  Serial.print("BUTTON_MINUS: ");
  Serial.println(BUTTON_MINUS);
  Serial.print("BUTTON_SET: ");
  Serial.println(BUTTON_SET);
  Serial.print("CLOCK_INTERRUPT_PIN: ");
  Serial.println(CLOCK_INTERRUPT_PIN);
}

void selectDisplay()
{
  Serial.println("Display ausgewählt");
  Serial.println("deaktivieren SD_CS: " + String(TFT_CS));
  digitalWrite(SD_CS, HIGH); // SD-Karte deaktivieren
  Serial.println("aktivieren TFT_CS: " + String(SD_CS));
  digitalWrite(TFT_CS, LOW); // Display aktivieren
}

void selectSDCard()
{
  Serial.println("SD-Karte ausgewählt");
  Serial.println("deaktivieren TFT_CS: " + String(SD_CS));
  digitalWrite(TFT_CS, HIGH); // Display deaktivieren
  Serial.println("aktivieren SD_CS: " + String(TFT_CS));
  digitalWrite(SD_CS, LOW); // SD-Karte aktivieren
}

void testOLED()
{
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

String currentTime()
{
  // Get the current time from the RTC
  String currentTime = "";
  if (rtc.now().hour() < 10)
    currentTime = "0"; // Add leading zero for hour
  currentTime += String(rtc.now().hour(), DEC) + ":";
  if (rtc.now().minute() < 10)
    currentTime += "0";                                 // Add leading zero for minute
  currentTime += String(rtc.now().minute(), DEC) + ":"; // Add minute with leading zero
  if (rtc.now().second() < 10)
    currentTime += "0";                           // Add leading zero for second
  currentTime += String(rtc.now().second(), DEC); // Add second with leading zero
  return currentTime;
}

String currentDate()
{
  // Get the current time from the RTC
  String currentDate;
  currentDate = String(rtc.now().year(), DEC) + "-";
  if (rtc.now().month() < 10)
    currentDate += "0";                                // Add leading zero for month
  currentDate += String(rtc.now().month(), DEC) + "-"; // Add month with leading zero
  if (rtc.now().day() < 10)
    currentDate += "0";                        // Add leading zero for day
  currentDate += String(rtc.now().day(), DEC); // Add day with leading zero);
  return currentDate;
}

void displayRpm(int rpm)
{
  tft.fillScreen(TFT_BLACK); // clear screen
  tft.setCursor(10, 15);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(4);
  tft.setTextWrap(true);
  tft.print(rpm);
}

void displayTimestamp()
{
  // tft.fillScreen(ST77XX_BLACK); // clear screen
  tft.setCursor(10, 70);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.print(currentTime());
  tft.setCursor(110, 70);
  tft.print("UTC");
  tft.setTextSize(1);
  tft.setCursor(25, 110);
  tft.print("press SET to set time");
}

void monitorDebug()
{
  Serial.print(currentTime());
  Serial.print(" ");
  Serial.print(currentDate());
  Serial.print(" ");
  Serial.print("Rpm_Count: ");
  Serial.println(Rpm_Count);
  Serial.print("Rpm: ");
  Serial.println(Rpm);
}

File openLogFile()
{
  String fileName = "/" + currentDate() + ".csv"; // Dateiname basierend auf dem aktuellen Datum
  bool fileExists = SD.exists(fileName);          // Prüfen, ob die Datei bereits existiert
  File file = SD.open(fileName, FILE_APPEND);     // Datei im Anhänge-Modus öffnen
  if (!file)
  {
    Serial.println("Fehler beim Öffnen der Datei: " + fileName);
  }
  else
  {
    Serial.println("Datei geöffnet: " + fileName);
    if (!fileExists)
    {
      // Schreibe Spaltenüberschriften, wenn die Datei neu erstellt wurde
      file.println("date,time,rpm");
      Serial.println("Spaltenüberschriften hinzugefügt.");
    }
  }
  return file;
}

void logData(File file, int rpm)
{
  if (file)
  {
    String logEntry = currentDate() + "," + currentTime() + "," + String(rpm);
    file.println(logEntry); // Daten in die Datei schreiben
    Serial.print("Daten geschrieben: ");
    Serial.println(logEntry);
  }
  else
  {
    Serial.println("Datei ist nicht geöffnet!");
  }
}

void closeLogFile(File file)
{
  if (file)
  {
    file.close();
    Serial.println("Datei geschlossen.");
  }
}

void displaySDCardWarning()
{
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

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup gestartet");
  // delay(2000); // Warten auf den Serial Monitor

  // Pin-Modus für die Buttons
  pinMode(BUTTON_PLUS, INPUT_PULLUP);
  pinMode(BUTTON_MINUS, INPUT_PULLUP);
  pinMode(BUTTON_SET, INPUT_PULLUP);

  // Interrupt für die "SET"-Taste konfigurieren
  attachInterrupt(digitalPinToInterrupt(BUTTON_SET), handleSetButtonInterrupt, FALLING);

  debugPinSettings(); // Pin-Einstellungen debuggen
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // RTC initialisieren
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC!");
    Serial.flush();
    while (1)
      delay(10);
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
  delay(3000);                // Warten auf den Serial Monitor
  // SD-Karte initialisieren
  Serial.println("Versuche, SD-Karte zu initialisieren...");
  selectSDCard();
  // SD-Karten initialisieren
  while (!SD.begin(SD_CS, tft.getSPIinstance()))
  {
    Serial.println("SD-Karte konnte nicht initialisiert werden!");
    delay(1000); // Warten auf den Serial Monitor
    selectDisplay();
    displaySDCardWarning();
    selectSDCard;
  }

  digitalWrite(SD_CS, HIGH); // SD-Karte deaktivieren

  // RPM sensor initialization
  setLed(true, LED_PIN);
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP); // Setze den IR-Sensor-Pin als Eingang mit Pull-Up Widerstand
  pinMode(LED_PIN, OUTPUT);

  Serial.println("Rpm Sensor gestartet");

  Rpm_Count = 0;                 // Initialisiere den Zähler
  lastTime = millis();           // Initialisiere die letzte Zeitmessung
  lastSecondRpmCount = millis(); // Initialisiere lastSecondRpmCount
  Rpm_Count_LastSecond = 0;      // Initialisiere Rpm_Count_LastSecond
  lastOutputTime = millis();

  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), Rpm_isr, FALLING); // Interrupt bei fallender Flanke
  setLed(false, LED_PIN);
  Serial.println("Rpm Sensor bereit");
  selectDisplay();
  // setupRTCWithButtons();      // RTC mit Tasten einstellen
  digitalWrite(TFT_CS, HIGH); // Display deaktivieren
  debugPinSettings();         // Pin-Einstellungen debuggen
  debugButtonSettings();      // Button-Einstellungen debuggen

  Serial.println("Setup abgeschlossen");
}

void loop()
{
  unsigned long currentTime = millis();

  // Überprüfen, ob der Interrupt für die "SET"-Taste ausgelöst wurde
  if (startRTCSetup)
  {
    noInterrupts();        // Interrupts deaktivieren, während das Flag bearbeitet wird
    startRTCSetup = false; // Flag zurücksetzen
    interrupts();          // Interrupts wieder aktivieren

    // Interrupt für die "SET"-Taste deaktivieren
    detachInterrupt(digitalPinToInterrupt(BUTTON_SET));

    Serial.println("SET-Taste gedrückt, RTC-Einstellung starten...");
    setupRTCWithButtons(); // Funktion zum Einstellen der RTC aufrufen
    Serial.println("RTC-Einstellung abgeschlossen.");
    // Verzögerung einfügen, um erneutes Auslösen zu verhindern
    delay(500); // 500 ms warten

    // Interrupt wieder aktivieren
    attachInterrupt(digitalPinToInterrupt(BUTTON_SET), handleSetButtonInterrupt, FALLING);
  }

  // Überprüfen, ob es Zeit für die nächste Messung ist
  if (currentTime - lastOutputTime >= measurementTime * 1000)
  {
    unsigned long impulseCount = Rpm_Count - Rpm_Count_LastSecond;
    Rpm = ((int)impulseCount * 60) / measurementTime / TRIGGERS_PER_REV;

    lastOutputTime = currentTime;
    Rpm_Count_LastSecond = Rpm_Count;

    if (TEST)
    {
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
    closeLogFile(logFile);        // Datei schließen
  }
}
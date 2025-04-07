/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete instructions at https://RandomNerdTutorials.com/esp32-ds3231-real-time-clock-arduino/
*********/

// based on the RTClib: implementation of an alarm using DS3231 https://github.com/adafruit/RTClib

// OLED display library
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

#define TRIGGERS_PER_REV 1 // Anzahl der Impulse pro Umdrehung

#define IR_SENSOR_PIN    15 // GPIO15: Infrarotsensor Pin
#define LED_PIN           2 // GPIO2: Kontroll-LED Pin
#define TFT_MOSI 23 // GPIO Standard-MOSI-Pin
#define MISO 19 // GPIO Standard-MOSI-Pin
#define TFT_SCLK 18 // GPIO Standard-SCLK-Pin
#define TFT_CS 5    // GPIO Chip Select für das Display
#define TFT_DC 2    // GPIO Data/Command (A0)
#define TFT_RST 4   // GPIO Reset
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

void selectDisplay() {
    digitalWrite(SD_CS, HIGH);  // SD-Karte deaktivieren
    digitalWrite(TFT_CS, LOW);  // Display aktivieren
}

void selectSDCard() {
    digitalWrite(TFT_CS, HIGH); // Display deaktivieren
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
void setup() {
    Serial.begin(115200);
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

    // // OLED display initialization
    // Serial.print(F("init 1.8 tft screen"));
    // tft.initR(INITR_BLACKTAB); // Init ST7735S chip, black tab
    // tft.setRotation(1);       // Set display to landscape (Querformat)
    // Serial.println(F("Initialized"));
    // if (TEST) testOLED();


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

    // SD-Karte initialisieren
    Serial.println("Versuche, SD-Karte zu initialisieren...");
    selectSDCard();
    if (!SD.begin(SD_CS)) {
      Serial.println("Versuche, Display zu initialisieren...");
      Serial.println("SD-Karte konnte nicht initialisiert werden!");
      while (1);
    }
    Serial.println("SD-Karte erfolgreich initialisiert.");
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
}

void loop() {
    unsigned long currentTime = millis();
    if (currentTime - lastOutputTime >= measurementTime * 1000) {
        unsigned long impulseCount = Rpm_Count - Rpm_Count_LastSecond;
        Rpm = ((int)impulseCount * 60) / measurementTime / TRIGGERS_PER_REV;

        lastOutputTime = currentTime;
        Rpm_Count_LastSecond = Rpm_Count;

        if (TEST) monitorDebug();

        // Display aktivieren und Daten anzeigen
        selectDisplay();
        displayRpm(Rpm);
        displayTimestamp();
        digitalWrite(TFT_CS, HIGH); // Display deaktivieren

        // SD-Karte aktivieren (falls benötigt)
        selectSDCard();
        // Hier kannst du Daten auf die SD-Karte schreiben
        digitalWrite(SD_CS, HIGH); // SD-Karte deaktivieren
    }
}

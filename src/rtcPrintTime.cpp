/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete instructions at https://RandomNerdTutorials.com/esp32-ds3231-real-time-clock-arduino/
*********/

// based on the RTClib: implementation of an alarm using DS3231 https://github.com/adafruit/RTClib

// OLED display library
#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#define TFT_CS 5    // Chip Select
#define TFT_RST 4   // Reset
#define TFT_DC 2   // Data/Command (A0)
#define TFT_MOSI 23 // SDA (MOSI)
#define TFT_SCLK 18 // SCK

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// based on the RTClib: implementation of an alarm using DS3231 https://github.com/adafruit/RTClib
// DS3231 RTC library
#include <RTClib.h>
RTC_DS3231 rtc;

bool TEST = true; // set to true to enable debug output
int rpm = 0; // variable to store the RPM value

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 4

// LED for visual indication
const int ledPin = 2;


String currentTime(){
  // Get the current time from the RTC
  String currentTime ="";
  if (rtc.now().hour() < 10) currentTime += "0"; // Add leading zero for hour
  currentTime = String(rtc.now().hour(), DEC) + ":";
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

void printRpm(int rpm) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(10, 20);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(5);
    tft.setTextWrap(true);
    tft.print(rpm);
}

void printTimestamp() {
    // tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(10, 75);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(3);
    tft.setTextWrap(true);
    tft.print(currentTime());
    tft.setTextSize(2);
    tft.setCursor(25, 110);
    tft.print(currentDate());
}

void printDebug() {
    Serial.print("Current time: ");
    Serial.print(rtc.now().hour(), DEC);
    Serial.print(":");
    Serial.print(rtc.now().minute(), DEC);
    Serial.print(":");
    Serial.print(rtc.now().second(), DEC);
    Serial.print(" ");
    Serial.print("Current date: ");
    Serial.print(rtc.now().year(), DEC);
    Serial.print("-");
    Serial.print(rtc.now().month(), DEC);
    Serial.print("-");
    Serial.println(rtc.now().day(), DEC);
}
void setup() {
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);

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

    // OLED display initialization
    Serial.begin(115200);
    Serial.print(F("init 1.8 tft screen"));
    tft.initR(INITR_BLACKTAB); // Init ST7735S chip, black tab
    tft.setRotation(1);       // Set display to landscape (Querformat)
    Serial.println(F("Initialized"));
}



void loop() {
  rpm++;
  // print current date and time
  if (TEST) printDebug();
  printRpm(rpm);
  printTimestamp(); 
  
  delay(1000);
}

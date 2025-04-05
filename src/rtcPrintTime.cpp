/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete instructions at https://RandomNerdTutorials.com/esp32-ds3231-real-time-clock-arduino/
*********/

// Example based on the RTClib: implementation of an alarm using DS3231 https://github.com/adafruit/RTClib
#include <RTClib.h>
RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 4

// LED for visual indication
const int ledPin = 2;


void printCurrentTime(){
  // Get the current time from the RTC
  DateTime now = rtc.now();
  
  // Getting each time field in individual variables
  // And adding a leading zero when needed;
  String yearStr = String(now.year(), DEC);
  String monthStr = (now.month() < 10 ? "0" : "") + String(now.month(), DEC);
  String dayStr = (now.day() < 10 ? "0" : "") + String(now.day(), DEC);
  String hourStr = (now.hour() < 10 ? "0" : "") + String(now.hour(), DEC); 
  String minuteStr = (now.minute() < 10 ? "0" : "") + String(now.minute(), DEC);
  String secondStr = (now.second() < 10 ? "0" : "") + String(now.second(), DEC);
  String dayOfWeek = daysOfTheWeek[now.dayOfTheWeek()];

  // Complete time string
  String formattedTime = dayOfWeek + ", " + yearStr + "-" + monthStr + "-" + dayStr + " " + hourStr + ":" + minuteStr + ":" + secondStr;

  // Print the complete formatted time
  Serial.println(formattedTime);
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

    // RTC-Setup fortsetzen
    rtc.disable32K();
    pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);

    rtc.clearAlarm(1);
    rtc.clearAlarm(2);
    rtc.writeSqwPinMode(DS3231_OFF);
    rtc.disableAlarm(2);

}

void loop() {
  // print current date and time
  printCurrentTime();

  Serial.println();
  delay(2000);
}

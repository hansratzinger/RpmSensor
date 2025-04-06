// RpmSensor.cpp
// measures RPM using an IR sensor 
// and outputs the RPM value every second
// to .csv file on the SD card
//
// HR 2025-02-06  1.0.0  Initial version
// HR 2025-02-08  1.1.1  Paket ID for RPM added
// HR 2025-02-25  1.1.2  Internal LED instead of RED_LED
// HR 2025-03-01  2.0.0  Removal FDRS, now storage on SD card, adding RTC clock
// ---------------------------------------------------------

// Status LED 
// #define GREEN_LED  25    // Grüne LED an GPIO25 (output-fähig)
// #define RED_LED    26    // Rote LED an GPIO26 (output-fähig)


#include <Arduino.h>

#define IR_SENSOR_PIN    15 // GPIO15: Infrarotsensor Pin
#define LED_PIN           2 // GPIO2: Kontroll-LED Pin

#define TRIGGERS_PER_REV 1 // Anzahl der Impulse pro Umdrehung

volatile unsigned long Rpm_Count; // Zähler für Interrupts
float Rpm;                // Variable für die berechnete Drehzahl
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
    digitalWrite(LED_PIN, HIGH); // LED einschalten
    delay(1);                     // Kurze Verzögerung, um das Aufleuchten sichtbar zu machen
    digitalWrite(LED_PIN, LOW);  // LED ausschalten
  }
}

// LED Funktion vereinfacht
void setLed(bool state, uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, state);
}


void setup() {
  setLed(true, LED_PIN);
  Serial.begin(115200);
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

  // Gib die Ergebnisse nur alle 'measurementTime' Sekunden aus
  if (currentTime - lastOutputTime >= measurementTime * 1000) {
    // Berechne die Anzahl der Impulse in der Messzeit
    unsigned long impulseCount = Rpm_Count - Rpm_Count_LastSecond;

    // Berechne die RPM basierend auf der Anzahl der Impulse pro Sekunde
    Rpm = ((float)impulseCount * 60) / measurementTime / TRIGGERS_PER_REV;

    Serial.print(">RPM:"); 
    Serial.println(Rpm);
    boardTime = (float)currentTime / 1000;

    lastOutputTime = currentTime;
    Rpm_Count_LastSecond = Rpm_Count;
    
  }
}
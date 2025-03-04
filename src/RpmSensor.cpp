#include <Arduino.h>

#define IR_SENSOR_PIN 15    // GPIO15: Infrarotsensor Pin
#define LED_PIN 2           // GPIO2: Beispiel LED-Pin

volatile unsigned long RPM_Count; // Zähler für Interrupts
unsigned long RPM;                // Variable für die berechnete Drehzahl
unsigned long lastTime;           // Variable für die letzte Zeitmessung
volatile unsigned long lastInterruptTime = 0;  // Zeit des letzten Interrupts

const int numReadings = 10;         // Anzahl der Messwerte für den gleitenden Durchschnitt
unsigned long readings[numReadings];  // Array für die Messwerte
int readIndex = 0;                  // Index des aktuellen Messwerts
unsigned long total = 0;                  // Summe der Messwerte
unsigned long averageRPM = 0;             // Durchschnittliche Drehzahl
unsigned long averageRPM2 = 0;             // Durchschnittliche Drehzahl

void IRAM_ATTR rpm_isr() {
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 1) { // Entprellzeit von 1ms
    RPM_Count++;
    lastInterruptTime = interruptTime;
    digitalWrite(LED_PIN, HIGH); // LED einschalten
    delay(1);                     // Kurze Verzögerung, um das Aufleuchten sichtbar zu machen
    digitalWrite(LED_PIN, LOW);  // LED ausschalten
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP); // Setze den IR-Sensor-Pin als Eingang mit Pull-Up Widerstand
  pinMode(LED_PIN, OUTPUT);          // Setze den LED-Pin als Ausgang
  Serial.println("RPM Sensor gestartet");

  RPM_Count = 0;          // Initialisiere den Zähler
  lastTime = millis();    // Initialisiere die letzte Zeitmessung

  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), rpm_isr, FALLING); // Interrupt bei fallender Flanke

  // Initialisiere das Array mit 0
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long timeInterval = currentTime - lastTime;

  if (timeInterval >= 333) { // Berechne die Drehzahl jede 1/3 Sekunde
    RPM = (RPM_Count * 60000) / timeInterval / 3; // Berechne die Drehzahl in U/min

    // Subtrahiere den ältesten Messwert
    total = total - readings[readIndex];
    // Lese den neuen Messwert
    readings[readIndex] = RPM;
    // Addiere den neuen Messwert
    total = total + readings[readIndex];
    // Erhöhe den Index
    readIndex = (readIndex + 1) % numReadings;
    // Berechne den Durchschnitt
    averageRPM = total / numReadings;
    averageRPM2 = (0.7 * averageRPM) + (0.3 * RPM); // 70% des alten Durchschnitts + 30% des neuen Werts

    RPM_Count = 0;      // Setze den Zähler zurück
    lastTime = currentTime; // Aktualisiere die letzte Zeitmessung

    // Wenn die RPM niedrig ist oder seit dem letzten Interrupt eine lange Zeit vergangen ist, setze den Zähler zurück
    if (averageRPM < 100 || (currentTime - lastInterruptTime > 333)) {
      RPM_Count = 0;
      total = 0;
      for (int i = 0; i < numReadings; i++) {
        readings[i] = 0;
      }
    }
    // Serial.print(">RPM:");
    // Serial.println(averageRPM);
    Serial.print( ">RPM2:");
    Serial.println(averageRPM2);
  } 
}
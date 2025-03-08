// RpmSensor.cpp
// measures RPM using an IR sensor 
// and outputs the RPM value every second
// based on FDRS library
// FDRS Node sends the RPM value to the FDRS gateway
// FARM DATA RELAY SYSTEM
// Developed by Timm Bogner (timmbogner@gmail.com) in Urbana, Illinois, USA.
//
// HR 2025-02-06  1.0.0  Initial version
// HR 2025-02-08  1.1.0  Paket ID for RPM added


#include <Arduino.h>
#include "fdrs_node_config.h"
#include <fdrs_node.h>

#define IR_SENSOR_PIN    15 // GPIO15: Infrarotsensor Pin
#define LED_PIN           2 // GPIO2: Kontroll-LED Pin
#define RPM              20 // RPM (Iterations genannt in FDRS)
#define RPM_PAKETSEND_ID 16 // RPM (Iterations genannt in FDRS)


volatile unsigned long Rpm_Count; // Zähler für Interrupts
float Rpm;                // Variable für die berechnete Drehzahl
unsigned long lastTime;           // Variable für die letzte Zeitmessung
volatile unsigned long lastInterruptTime = 0;  // Zeit des letzten Interrupts

unsigned long lastOutputTime = 0; // Zeitpunkt der letzten Ausgabe
unsigned long lastSecondRpmCount = 0;
unsigned long Rpm_Count_LastSecond = 0;
float rpmPacketSend_ID = 0;
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

void sendFDRS(float data1, float data2) {   // Sendet die RPM-Werte an den FDRS-Gateway 
  loadFDRS(data1, RPM);
  loadFDRS(data2, RPM_PAKETSEND_ID);
  // DBG(sendFDRS()); // Debugging 
  if (sendFDRS()) {
    DBG("Big Success!");
  } else {
    DBG("Nope, not so much.");
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP); // Setze den IR-Sensor-Pin als Eingang mit Pull-Up Widerstand
  pinMode(LED_PIN, OUTPUT);          // Setze den LED-Pin als Ausgang

  beginFDRS(); // Initialisiere FDRS

  Serial.println("Rpm Sensor gestartet");

  Rpm_Count = 0;          // Initialisiere den Zähler
  lastTime = millis();    // Initialisiere die letzte Zeitmessung
  lastSecondRpmCount = millis(); // Initialisiere lastSecondRpmCount
  Rpm_Count_LastSecond = 0; // Initialisiere Rpm_Count_LastSecond
  lastOutputTime = millis();

  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), Rpm_isr, FALLING); // Interrupt bei fallender Flanke
}

void loop() {
  unsigned long currentTime = millis();

  // Gib die Ergebnisse nur alle 'measurementTime' Sekunden aus
  if (currentTime - lastOutputTime >= measurementTime * 1000) {
    // Berechne die Anzahl der Impulse in der Messzeit
    unsigned long impulseCount = Rpm_Count - Rpm_Count_LastSecond;

    // Berechne die RPM basierend auf der Anzahl der Impulse pro Sekunde
    Rpm = ((float)impulseCount * 60) / measurementTime / 3;

    Serial.print(">RPM:"); // Formatierung für die Ausgabe auf Teleplot (optional)
    Serial.println(Rpm);
    rpmPacketSend_ID = rpmPacketSend_ID + 1;
    sendFDRS(Rpm, rpmPacketSend_ID); // Sendet die RPM-Werte an den FDRS-Gateway

    lastOutputTime = currentTime;
    Rpm_Count_LastSecond = Rpm_Count;
    rpmPacketSend_ID = (rpmPacketSend_ID > 115200) ? 0 : rpmPacketSend_ID; // prevens overflow
  }
}
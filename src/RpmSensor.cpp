#include <Arduino.h>

#define IR_SENSOR_PIN 15    // GPIO15: Infrarotsensor Pin
#define LED_PIN 2           // GPIO2: Beispiel LED-Pin

volatile unsigned long RPM_Count; // Zähler für Interrupts
float RPM;                // Variable für die berechnete Drehzahl
unsigned long lastTime;           // Variable für die letzte Zeitmessung
volatile unsigned long lastInterruptTime = 0;  // Zeit des letzten Interrupts

unsigned long lastOutputTime = 0; // Zeitpunkt der letzten Ausgabe
unsigned long lastSecondRPMCount = 0;
unsigned long RPM_Count_LastSecond = 0;

const int measurementTime = 1; // Messzeit in Sekunden

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
  lastSecondRPMCount = millis(); // Initialisiere lastSecondRPMCount
  RPM_Count_LastSecond = 0; // Initialisiere RPM_Count_LastSecond
  lastOutputTime = millis();

  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), rpm_isr, FALLING); // Interrupt bei fallender Flanke
}

void loop() {
  unsigned long currentTime = millis();

  // Gib die Ergebnisse nur alle 'measurementTime' Sekunden aus
  if (currentTime - lastOutputTime >= measurementTime * 1000) {
    // Berechne die Anzahl der Impulse in der Messzeit
    unsigned long impulseCount = RPM_Count - RPM_Count_LastSecond;

    // Berechne die RPM basierend auf der Anzahl der Impulse pro Sekunde
    RPM = ((float)impulseCount * 60) / measurementTime / 3;

    Serial.print(">RPM1sImp:");
    Serial.println(RPM);

    lastOutputTime = currentTime;
    RPM_Count_LastSecond = RPM_Count;
  }
}
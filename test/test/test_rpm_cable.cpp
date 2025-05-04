// Test-Programm für lange Kabelstrecken
#include <Arduino.h>

#define IR_SENSOR_PIN 15
#define LED_PIN 2

volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;

// Interrupt-Handler mit angepasster Entprellzeit
void IRAM_ATTR sensorISR() {
  unsigned long currentTime = micros();
  
  // Etwas längere Entprellzeit für 10µF
  if (currentTime - lastPulseTime < 150000) {  // 150ms
    return;
  }
  
  pulseCount++;
  lastPulseTime = currentTime;
  digitalWrite(LED_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  pinMode(IR_SENSOR_PIN, INPUT);  // Mit Pull-up
  pinMode(LED_PIN, OUTPUT);
  
  // Teste unterschiedliche Interrupt-Modi
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), sensorISR, FALLING);
  
  Serial.println("Sensor-Test über langes Kabel gestartet");
}

void loop() {
  static unsigned long lastCount = 0;
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();
  
  // LED nach 50ms ausschalten
  if (digitalRead(LED_PIN) == HIGH && (currentTime - lastPulseTime/1000) > 50) {
    digitalWrite(LED_PIN, LOW);
  }
  
  // Statusupdate alle 2 Sekunden
  if (currentTime - lastPrintTime >= 2000) {
    unsigned long newPulses = pulseCount - lastCount;
    
    Serial.print("Neue Impulse: ");
    Serial.print(newPulses);
    Serial.print(" | Aktueller Status: ");
    Serial.println(digitalRead(IR_SENSOR_PIN) ? "HIGH" : "LOW");
    
    lastCount = pulseCount;
    lastPrintTime = currentTime;
  }
}
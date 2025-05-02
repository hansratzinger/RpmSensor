// -------------------------------------------------
// GPIO 15 Impulstest für AZdelivery IR Obstacle Avoidance Sensor
// -------------------------------------------------

#include <Arduino.h>

// Pin-Definitionen
#define IR_SENSOR_PIN 15  // GPIO15: IR Sensor
#define LED_PIN 2         // GPIO2: Onboard LED

// Globale Zählvariablen
volatile unsigned long totalPulses = 0;
unsigned long lastPulseCount = 0;
unsigned long lastOutputTime = 0;
unsigned long lastLedTime = 0;

// Für Zeit zwischen Impulsen
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;

// Flag für neue Impulse
volatile bool newPulse = false;

// Interrupt Service Routine - reagiert auf FALLING weil der Sensor aktiv-LOW ist
void IRAM_ATTR sensorISR() {
  unsigned long currentTime = micros();
  
  // Debounce mit 10ms (dieser Sensor braucht ein längeres Debounce)
  static unsigned long lastTime = 0;
  if (currentTime - lastTime < 10000) {  // 10ms Entprellzeit
    return;
  }
  lastTime = currentTime;
  
  // Impulszählung und Intervallberechnung
  if (lastPulseTime > 0) {
    pulseInterval = currentTime - lastPulseTime;
  }
  lastPulseTime = currentTime;
  
  totalPulses++;
  newPulse = true;
  
  // LED einschalten zur Visualisierung
  digitalWrite(LED_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\nAZdelivery IR Obstacle Sensor Test");
  Serial.println("==================================");
  
  // Pin-Modi konfigurieren (Wichtig: Pullup für IR Sensor deaktiviert)
  pinMode(IR_SENSOR_PIN, INPUT); // KEIN Pullup für diesen Sensor
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initial Pin-Status ausgeben
  Serial.print("Initialer GPIO 15 Status: ");
  Serial.println(digitalRead(IR_SENSOR_PIN) ? "HIGH (kein Objekt)" : "LOW (Objekt erkannt)");
  
  // LED 3x blinken
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  // Interrupt einrichten - FALLING weil der Sensor aktiv-LOW ist
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), sensorISR, FALLING);
  
  // Start-Informationen
  Serial.println("\nImpulszählung beginnt...");
  Serial.println("Drehen Sie das Rad oder bewegen Sie Objekte vor dem Sensor");
  Serial.println("Die LED blinkt bei jedem erkannten Impuls\n");
  
  lastOutputTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // 1. LED-Steuerung: Nach 50ms ausschalten, wenn ein Impuls erkannt wurde
  if (digitalRead(LED_PIN) == HIGH && currentTime - lastLedTime > 50) {
    digitalWrite(LED_PIN, LOW);
  }
  
  // 2. Wenn ein neuer Impuls erkannt wurde
  if (newPulse) {
    newPulse = false;
    lastLedTime = currentTime;
    
    // Intervallzeit zwischen Impulsen ausgeben
    if (pulseInterval > 0) {
      float intervalMs = pulseInterval / 1000.0;
      
      Serial.print("Impuls! Intervall: ");
      Serial.print(intervalMs, 2);
      Serial.println(" ms");
      
      // RPM-Berechnung
      float rpm = (60.0 * 1000000.0) / (pulseInterval * 2);
      Serial.print("-> RPM (bei 2 Impulsen/Umdrehung): ");
      Serial.println(rpm, 1);
    }
  }
  
  // 3. Zusammenfassung jede Sekunde ausgeben
  if (currentTime - lastOutputTime >= 1000) {
    // Aktuelle Impulse pro Sekunde berechnen
    unsigned long newPulses = totalPulses - lastPulseCount;
    lastPulseCount = totalPulses;
    
    Serial.println("\n=== Sekunden-Update ===");
    Serial.print("GPIO 15 Status: ");
    Serial.println(digitalRead(IR_SENSOR_PIN) ? "HIGH (kein Objekt)" : "LOW (Objekt erkannt)");
    Serial.print("Impulse pro Sekunde: ");
    Serial.println(newPulses);
    Serial.print("Gesamtimpulse: ");
    Serial.println(totalPulses);
    
    if (newPulses > 0) {
      float rpm = newPulses * 30.0f;  // 60/2
      Serial.print("Durchschnittliche RPM: ");
      Serial.println(rpm, 1);
    }
    
    Serial.println("========================");
    
    lastOutputTime = currentTime;
  }
  
  // 4. Kurze Pause
  delay(10);
}
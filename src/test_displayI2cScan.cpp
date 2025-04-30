// -------------------------------------------------
// OLED Display Test mit I2C Scanner
// -------------------------------------------------
// Überprüft alle I2C-Geräte und testet zwei OLED-Displays
// RNLI RPM Sensor System
// 2025-04-30 NK
// -------------------------------------------------

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// I2C Pins
#define SDA_PIN 21
#define SCL_PIN 22

// Erwartete Display-Adressen (in 7-Bit-Format)
#define OLED1_ADDR_7BIT 0x3C  // = 0x78 in 8-Bit-Format
#define OLED2_ADDR_7BIT 0x3D  // = 0x7A in 8-Bit-Format

// LED-Pin für visuelles Feedback
#define LED_PIN 12

// Beide Displays initialisieren, ohne konkrete Adresse
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display1(U8G2_R0, U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display2(U8G2_R0, U8X8_PIN_NONE);

// Array für gefundene I2C-Adressen
byte foundAddresses[10];
byte addressCount = 0;

// Funktion zur Konvertierung von 7-Bit- zu 8-Bit-Adresse
byte convert7to8Bit(byte addr) {
  return addr << 1;
}

// Funktion zur Konvertierung von 8-Bit- zu 7-Bit-Adresse
byte convert8to7Bit(byte addr) {
  return addr >> 1;
}

// I2C-Scanner-Funktion, gibt die Anzahl gefundener Geräte zurück
byte scanI2C() {
  byte error, address;
  byte count = 0;
  
  Serial.println("Scanne I2C-Bus...");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if(error == 0) {
      Serial.print("I2C-Gerät gefunden an Adresse 0x");
      if(address < 16) {
        Serial.print("0");
      }
      
      Serial.print(address, HEX);
      Serial.print(" (7-Bit) / 0x");
      
      byte address8bit = convert7to8Bit(address);
      if(address8bit < 16) {
        Serial.print("0");
      }
      
      Serial.print(address8bit, HEX);
      Serial.println(" (8-Bit)");
      
      if(count < 10) {
        foundAddresses[count] = address;
        count++;
      }
      
      // Kurzes Blinken der LED zur Bestätigung
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
    else if(error == 4) {
      Serial.print("Unbekannter Fehler an Adresse 0x");
      if(address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  
  if(count == 0) {
    Serial.println("Keine I2C-Geräte gefunden!");
  } else {
    Serial.print("Gefunden: ");
    Serial.print(count);
    Serial.println(" Gerät(e)");
  }
  
  return count;
}

// Testet das übergebene Display mit der angegebenen Adresse und ID
bool testDisplay(U8G2 &display, byte address8bit, const char* displayName) {
  Serial.print("Teste ");
  Serial.print(displayName);
  Serial.print(" an Adresse 0x");
  Serial.print(address8bit, HEX);
  Serial.println("...");
  
  display.begin();
  display.setI2CAddress(address8bit);
  
  // Test mit einfachem Text
  display.clearBuffer();
  display.setFont(u8g2_font_helvB12_tf);
  
  display.setCursor(0, 20);
  display.print(displayName);
  
  display.setCursor(0, 40);
  display.print("Addr: 0x");
  display.print(address8bit, HEX);
  
  display.setCursor(0, 60);
  display.print("Test OK!");
  
  display.sendBuffer();
  
  Serial.print(displayName);
  Serial.println(" Test abgeschlossen.");
  
  return true;
}

// Dreifach-Blink-Funktion für Statusanzeige
void blinkLED(int count) {
  for(int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Kurze Pause für die serielle Verbindung
  delay(1000);
  
  Serial.println("\n\nOLED Display Test mit I2C Scanner");
  Serial.println("====================================");
  
  // LED für Statusanzeige initialisieren
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // I2C-Bus initialisieren
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Schnelles Blinken zur Bestätigung des Programmstarts
  blinkLED(3);
  
  // I2C-Scan durchführen
  addressCount = scanI2C();
  
  if(addressCount == 0) {
    Serial.println("FEHLER: Keine I2C-Geräte gefunden!");
    while(1) {
      // Schnell blinken bei Fehler
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  // Überprüfen, ob die erwarteten Display-Adressen gefunden wurden
  bool display1Found = false;
  bool display2Found = false;
  
  for(byte i = 0; i < addressCount; i++) {
    if(foundAddresses[i] == OLED1_ADDR_7BIT) {
      display1Found = true;
    }
    if(foundAddresses[i] == OLED2_ADDR_7BIT) {
      display2Found = true;
    }
  }
  
  Serial.println("\nErgebnisse des I2C-Scans:");
  Serial.print("Display 1 (0x");
  Serial.print(OLED1_ADDR_7BIT, HEX);
  Serial.print("/0x");
  Serial.print(convert7to8Bit(OLED1_ADDR_7BIT), HEX);
  Serial.print("): ");
  Serial.println(display1Found ? "GEFUNDEN" : "NICHT GEFUNDEN");
  
  Serial.print("Display 2 (0x");
  Serial.print(OLED2_ADDR_7BIT, HEX);
  Serial.print("/0x");
  Serial.print(convert7to8Bit(OLED2_ADDR_7BIT), HEX);
  Serial.print("): ");
  Serial.println(display2Found ? "GEFUNDEN" : "NICHT GEFUNDEN");
  
  // Warnungen ausgeben für nicht gefundene Displays
  if(!display1Found) {
    Serial.println("WARNUNG: Display 1 nicht gefunden!");
  }
  if(!display2Found) {
    Serial.println("WARNUNG: Display 2 nicht gefunden!");
  }
  
  // Displays testen
  Serial.println("\nStarte Display-Tests...");
  
  // Teste alle gefundenen I2C-Geräte als potentielle Displays
  for(byte i = 0; i < addressCount; i++) {
    byte addr7bit = foundAddresses[i];
    byte addr8bit = convert7to8Bit(addr7bit);
    
    char displayName[20];
    
    if(addr7bit == OLED1_ADDR_7BIT) {
      sprintf(displayName, "Display 1");
    } else if(addr7bit == OLED2_ADDR_7BIT) {
      sprintf(displayName, "Display 2");
    } else {
      sprintf(displayName, "Unknown %d", i+1);
    }
    
    // Teste jedes gefundene Gerät als Display
    if(i == 0) {
      testDisplay(display1, addr8bit, displayName);
      delay(2000);
    } else if(i == 1) {
      testDisplay(display2, addr8bit, displayName);
      delay(2000);
    }
  }
  
  Serial.println("\nDisplay-Test abgeschlossen!");
}

void loop() {
  // Statusanzeige: 1x Blinken alle 3 Sekunden für normalen Betrieb
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  delay(2800);
}
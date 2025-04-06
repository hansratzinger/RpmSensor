#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#define TFT_CS 5    // Chip Select
#define TFT_RST 4   // Reset
#define TFT_DC 2   // Data/Command (A0)
#define TFT_MOSI 23 // SDA (MOSI)
#define TFT_SCLK 18 // SCK
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void drawtext(char *text, uint16_t color) {
    tft.setCursor(0, 50);
    tft.setTextColor(color);
    tft.setTextSize(2);
    tft.setTextWrap(true);
    tft.print(text);
}
void fillScreenBlink(uint16_t color1, uint16_t color2) {
    tft.fillScreen(color1);
    delay(1000);
    tft.fillScreen(color2);
}

void setup(void) {
    Serial.begin(115200);
    Serial.print(F("init 1.8 tft screen"));
    tft.initR(INITR_BLACKTAB); // Init ST7735S chip, black tab
    tft.setRotation(1);       // Set display to landscape (Querformat)
    Serial.println(F("Initialized"));
    // tft.fillScreen(ST77XX_BLACK);
    // drawtext(" No RMP recgnized!\n   2025", ST77XX_WHITE);
    // delay(5000);
    // fillScreenBlink(ST77XX_WHITE, ST77XX_RED);
    // delay(5000);
    // Serial.println("done");
    // delay(1000);
    // tft.writeLine(0, 0, 127, 159, ST77XX_RED);
}

void counter() {
    for (int i = 0; i < 1000; i++) {
        tft.fillScreen(ST77XX_BLACK);
        tft.setCursor(10, 20);
        tft.setTextColor(ST77XX_WHITE);
        tft.setTextSize(5);
        // tft.setTextWrap(true);
        tft.print(i);
        tft.setCursor(10, 80);
        tft.setTextSize(3);
        tft.print ("12:00:00");
        
        delay(1000);
    }
}


void loop() {
    counter();
    delay(100); 
}

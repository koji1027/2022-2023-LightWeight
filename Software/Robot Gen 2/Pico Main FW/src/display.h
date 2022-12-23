#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>

Adafruit_SSD1306 ssd1306(-1);

class Display {
   public:
    void begin();
};

void Display::begin() { ssd1306.begin(SSD1306_SWITCHCAPVCC, 0x3C); }

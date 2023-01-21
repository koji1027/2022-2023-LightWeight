
#include <Arduino.h>



class Display {
   public:
    void begin();
    void refresh_info();

   private:
};

void Display::begin {
    Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextColor(SSD1306_WHITE);
}
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#define LED_PIN 15
#define LED_NUM 32

Adafruit_NeoPixel led(LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);

void set_led() {
    for (int i = 0; i < LED_NUM; i++) {
        led.setPixelColor(i, led.Color(255, 255, 255));
    }
    led.setBrightness(255);
    led.show();
}
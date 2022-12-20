#ifndef LED_H
#define LED_H

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#define LED_PIN D15
#define LED_NUM 16

Adafruit_NeoPixel led(LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);

void set_led(int color[3], int brightness) {
    for (int i = 0; i < LED_NUM; i++) {
        led.setPixelColor(i, led.Color(color[0], color[1], color[2]));
    }
    led.setBrightness(brightness);
    led.show();
}

#endif
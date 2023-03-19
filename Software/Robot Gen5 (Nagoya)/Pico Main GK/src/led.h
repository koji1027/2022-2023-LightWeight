#ifndef LED_H
#define LED_H

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#define LED_PIN D15
#define LED_NUM 32

Adafruit_NeoPixel led(LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);

void set_led(uint8_t *color, uint8_t brightness)
{
        for (int i = 0; i < LED_NUM; i++)
        {
                led.setPixelColor(i, led.Color(color[0], color[1], color[2]));
        }
        led.setBrightness(brightness);
        led.show();
}

void init_led()
{
        for (int i = 0; i < LED_NUM; i++)
        {
                led.setPixelColor(i, led.Color(255, 0, 0));
        }
        led.setBrightness(200);
        led.show();
}

#endif
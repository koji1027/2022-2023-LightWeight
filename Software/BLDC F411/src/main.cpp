#include <Arduino.h>

HardwareSerial Serial1(PA10, PA9);
const int pwm[3] = {PA6, PA7, PB0};

void setup()
{
        // put your setup code here, to run once:
        pinMode(PA1, INPUT_ANALOG);
        Serial1.begin(115200);
        analogReadResolution(12);
}

void loop()
{
        // put your main code here, to run repeatedly:
        Serial1.println(analogRead(PA1));
        delay(100);
}
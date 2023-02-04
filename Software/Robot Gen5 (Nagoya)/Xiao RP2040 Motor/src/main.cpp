#include <Arduino.h>

#include "motor.h"

Motor motor;

void setup()
{
        // put your setup code here, to run once:
        motor.begin();
        Serial.begin(115200);
        delay(3000);
}

void loop()
{
        // put your main code here, to run repeatedly:
}

void setup1()
{
        // put your setup code here, to run once:
        Serial1.begin(115200);
}

void loop1()
{
        // put your main code here, to run repeatedly:
}
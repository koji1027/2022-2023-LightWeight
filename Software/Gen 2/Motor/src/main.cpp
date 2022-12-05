#include <Arduino.h>

#include "motor_control.h"

Motor motor;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial1.begin(115200);
    motor.begin();
}

void loop() {
    // put your main code here, to run repeatedly:
}
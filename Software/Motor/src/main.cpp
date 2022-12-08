#include <Arduino.h>

//#include "motor_control.h"

// Motor motor;
int pin[4][2] = {{0, 1}, {2, 3}, {4, 5}, {8, 9}};

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    // motor.begin();
    for (int i = 0; i < 4; i++) {
        pinMode(pin[i][0], OUTPUT);
        pinMode(pin[i][1], OUTPUT);
        digitalWrite(pin[i][0], LOW);
        digitalWrite(pin[i][1], LOW);
    }
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    digitalWrite(0, LOW);
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    delay(100);
}
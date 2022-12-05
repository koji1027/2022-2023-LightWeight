#include <Arduino.h>

#include "led.h"
#include "line.h"

void set_led();

Line line;
SerialPIO motor(22, 16, 32);

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    motor.begin(115200);
    // pinMode(15, OUTPUT);
    // led.begin();
    // line.begin();
}

void loop() {
    // set_led();
    // line.read();
    // line.print();
    motor.write(100);
    delay(100);
}
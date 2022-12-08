#include <Arduino.h>
#include <Wire.h>

#include "gyro.h"
#include "led.h"
#include "line.h"

// Line line;
SerialPIO motor(22, 16, 32);
Gyro gyro;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    gyro.begin();
    gyro.calibration();
    //  pinMode(15, OUTPUT);
    //  led.begin();
    //  line.begin();
}

void loop() {
    gyro.read();
    gyro.calcAngle();
    // delay(10);
    //  set_led();
    //  line.read();
    //  line.print();
    //  motor.write(100);
}

void setup1() { motor.begin(115200); }

void loop1() {
    int a = (gyro.angle + PI) * 100;
    byte data[2];
    data[0] = byte(a);
    data[1] = byte(a >> 8);
    motor.write(255);
    motor.write(data, 2);
    delay(10);
}
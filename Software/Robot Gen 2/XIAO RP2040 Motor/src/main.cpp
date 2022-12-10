#include <Arduino.h>

#include "motor.h"

Motor motor;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    motor.begin();
    delay(5000);
}

void loop() { motor.cal(); }

void setup1() { Serial1.begin(115200); }

void loop1() {
    if (Serial1.available() > 4) {
        int recv_data = Serial1.read();
        if (recv_data == 255) {
            int data[4];  //[0][1]:gyro, [2][3]:ir
            data[0] = Serial1.read();
            data[1] = Serial1.read();
            int a = data[0] + (data[1] << 8);
            motor.gyro_angle = (a / 100.0) - PI;
            data[2] = Serial1.read();
            data[3] = Serial1.read();
            int b = data[2] + (data[3] << 8);
            motor.go_angle = (b / 100.0) - PI;
        }
    }
}
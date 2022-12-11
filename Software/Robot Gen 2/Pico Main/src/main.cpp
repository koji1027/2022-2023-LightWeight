#include <Arduino.h>
#include <Wire.h>

#include "gyro.h"
#include "led.h"
#include "line.h"

SerialPIO motor(22, 16, 32);
Line line;
Gyro gyro;

float ir_angle = 0.0;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    gyro.begin();
    gyro.calibration();
    led.begin();
    line.begin();
}

void loop() {
    gyro.read();
    gyro.calcAngle();
    set_led();
    line.read();
    line.print();
}

void setup1() {
    motor.begin(115200);
    Serial1.begin(9600);
}

void loop1() {
    if (Serial1.available() > 2) {
        int recv_data = Serial1.read();
        if (recv_data == 255) {
            int data[2];
            data[0] = Serial1.read();
            data[1] = Serial1.read();
            int a = data[0] + (data[1] << 8);
            ir_angle = (a / 100.0) - PI;
        }
        // Serial.println(ir_angle);
    }

    int a = (gyro.angle + PI) * 100;
    int b = (ir_angle + PI) * 100;
    byte data[4];  //[0][1]:gyro, [2][3]:ir
    data[0] = byte(a);
    data[1] = byte(a >> 8);
    data[2] = byte(b);
    data[3] = byte(b >> 8);
    motor.write(255);
    motor.write(data, 4);
    delay(10);
}
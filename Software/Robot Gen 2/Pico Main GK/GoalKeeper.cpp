#include <Arduino.h>
#include <Wire.h>

#include "MPU6050/gyro.h"
// #include "gyro.h"
#include "led.h"
#include "line.h"

SerialPIO motor(22, 16, 32);
SerialPIO ir(1, 0, 32);
Line line;
Gyro gyro;

float ir_angle = 0.0;
int color[3] = {255, 255, 255};
int brightness = 255;
int distance = 0;
bool start_flag = false;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    gyro.begin();
    
    // gyro.calibration();
    line.begin();
    // gyro.begin();
    pinMode(D18, INPUT_PULLUP);  // 機能する
    /*pinMode(D19, INPUT_PULLUP);  // 機能しない
    pinMode(D20, INPUT_PULLUP);  // 機能する
    pinMode(D21, INPUT_PULLUP);  // 機能しない
    */
   /*
    while (!start_flag) {
        if (!digitalRead(D18)) {
            start_flag = true;
        }
    }
    */
    led.begin();
}

void loop() {
    
    gyro.getEuler();
    Serial.println(gyro.angle);
    // 
    // gyro.cal_vel();
    line.read();
    set_led(color, brightness);
}

void setup1() {
    start_flag = false;
    motor.begin(115200);
    ir.begin(115200);
}

void loop1() {
    ir.write(255);
            if (ir.available() > 2) {
                int recv_data = ir.read();
                if (recv_data == 255) {
                    int data[2];
                    data[0] = ir.read();
                    data[1] = ir.read();
                    int a = data[0] + (data[1] << 8);
                    ir_angle = (a / 100.0) - PI;
                    // Serial.println(ir_angle);
                } 
            }

            int a = (gyro.angle + PI) * 100;
            int b = (ir_angle + PI) * 100;
            int c = 0;  // 1:line, 0:ir
            if (line.entire_sensor_state == true) {
                b = (line.line_theta + PI) * 100;
                c = 1;
            } else {
                b = (ir_angle + PI) * 100;
                c = 0;
            }
            byte data[5];  //[0][1]:gyro, [2][3]:go
            data[0] = byte(a);
            data[1] = byte(a >> 8);
            data[2] = byte(b);
            data[3] = byte(b >> 8);
            data[4] = byte(c);
            motor.write(255);
            motor.write(data, 5);  
}
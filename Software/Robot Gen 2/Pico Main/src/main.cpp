#include <Arduino.h>
#include <Wire.h>

#include "gyro.h"
#include "led.h"
#include "line.h"

SerialPIO motor(22, 16, 32);
Line line;
Gyro gyro;

float ir_angle = 0.0;
float circulate_angle = 0.0;
void circulate();

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
    // Serial.println(line.line_theta);
    // line.print();
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
    circulate();
    Serial.print(ir_angle);
    Serial.print("\t");
    Serial.println(circulate_angle);

    int send_gyro = (gyro.angle + PI) * 100;
    int send_move = (ir_angle + PI) * 100;
    int c = 0;  // 1:line, 0:ir
    if (line.entire_sensor_state == true) {
        send_move = (line.line_theta + PI) * 100;
        c = 1;
    } else {
        send_move = (circulate_angle + PI) * 100;
        c = 0;
    }
    byte data[5];  //[0][1]:gyro, [2][3]:move
    data[0] = byte(send_gyro);
    data[1] = byte(send_gyro >> 8);
    data[2] = byte(send_move);
    data[3] = byte(send_move >> 8);
    data[4] = byte(c);
    motor.write(255);
    motor.write(data, 5);
}

void circulate(){
    if (ir_angle > 0.4){
        circulate_angle = ir_angle + (PI / 3);
        if (circulate_angle > PI){
            circulate_angle = circulate_angle - PI*2;
        }
    }
    else if (ir_angle < -0.4){
        circulate_angle = ir_angle - (PI / 3);
        if (circulate_angle < -PI){
            circulate_angle = circulate_angle + PI*2;
        }
    }
    else {
        circulate_angle = 0;
    }
}
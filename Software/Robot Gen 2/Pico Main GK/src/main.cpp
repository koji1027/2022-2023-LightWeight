#include <Arduino.h>
#include <Wire.h>

#include "MPU6050/gyro.h"
#include "led.h"
#include "line.h"

#define LINE_FLAG_GOAL 100

SerialPIO motor(22, 16, 32);
SerialPIO ir(1, 0, 32);
Line line;
Gyro gyro;

void circulate();
void linetrace();

float ir_angle = 0.0;
float circulate_angle = 0.0;
float linetrace_angle = 0.0;
int send_gyro = 0.0;
int send_move = 0.0;
int line_flag = 0;  // 左:1 右:2
int line_flag_count = 0;
int color[3] = {255, 255, 255};
int brightness = 255;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    gyro.begin();
    led.begin();
    line.begin();
}

void loop() {
    gyro.getEuler();
    set_led(color, brightness);
    line.read();
    // Serial.println(gyro.angle);
    // Serial.println(line.line_theta);
    // line.print();
}

void setup1() {
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
        }
         Serial.println(ir_angle);
    } 
    circulate();

    // Serial.println(gyro.angle);
    // Serial.println(line_flag);

    send_gyro = (gyro.angle + PI) * 100;
    send_move = (ir_angle + PI) * 100;
    int c = 0;  // 1:line, 0:ir
    if (line.entire_sensor_state == true) {
        send_move = (line.line_theta + PI) * 100;
        // linetrace();
        c = 1;
        line_flag_count = 0;
    } else {
        send_move = (circulate_angle + PI) * 100;
        c = 0;
        line_flag_count++;
        if (line_flag_count == LINE_FLAG_GOAL){
            line_flag = 0;
            line_flag_count = 0;
        }
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

void circulate() {
    if (ir_angle > PI / 6) {
        circulate_angle = ir_angle + (PI / 3);
        if (circulate_angle > PI) {
            circulate_angle = circulate_angle - PI * 2;
        }
    } else if (ir_angle < -PI / 6) {
        circulate_angle = ir_angle - (PI / 3);
        if (circulate_angle < -PI) {
            circulate_angle = circulate_angle + PI * 2;
        }
    } else {
        circulate_angle = ir_angle;
    }
}

void linetrace() {
    if ((line.line_theta > -PI / 4 && line.line_theta < PI / 4) ||
        line.line_theta > PI * 3 / 4 || line.line_theta < -PI * 3 / 4) {
        send_move = (line.line_theta + PI) * 100;
    } else if (line.line_theta < 0) {
        if (line_flag == 2) {
            send_move = -PI / 2 * 100;
        } else if (ir_angle < 0) {
            if (ir_angle > -PI / 3) {
                send_move = 0 * 100;
            } else if (ir_angle < -PI / 4) {
                send_move = PI * 100;
            }
            line_flag = 1;
        }
    } else if (line.line_theta > 0) {
        if (line_flag == 1) {
            send_move = PI / 2 * 100;
        } else if (ir_angle > 0) {
            if (ir_angle < PI / 3) {
                send_move = 0 * 100;
            } else if (ir_angle > PI / 4) {
                send_move = PI * 100;
            }
            line_flag = 2;
        }
    }
}
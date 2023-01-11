#include <Arduino.h>

#include "MPU6050/gyro.h"
#include "led.h"
#include "line.h"

#define DIST_BALL -20.0
#define B pow(0.7, 1.0 / 20.0)

SerialPIO motor(22, 16, 32);
SerialPIO ir(1, 0, 32);
Gyro gyro;
Line line;

float machine_angle = 0.0;
float ir_angle = 0.0;
float ir_radius = 0.0;
float move_angle = 0.0;
int ball_flag = 0;
bool start_flag = false;
int led_color[3] = {255, 255, 255};
int led_brightness = 50;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);

    pinMode(D18, INPUT_PULLUP);
    pinMode(D20, INPUT_PULLUP);
    pinMode(D21, INPUT_PULLUP);

    gyro.begin();
    line.begin();
    while (1) {
        set_led(led_color, led_brightness);
        gyro.getEuler();
        Serial.println(gyro.angle);
        delay(100);
    }
    delay(1000);
}

void loop() {
    // put your main code here, to run repeatedly:
    while (!start_flag) {
        if (!digitalRead(D18)) {
            start_flag = true;
        }
        delay(10);
    }
    // gyro.getEuler();
    if (!digitalRead(D21)) {
        start_flag = false;
    }
    set_led(led_color, led_brightness);
}
/*
void setup1() {
    motor.begin(115200);
    while (!motor) {
        delay(10);
    }
    ir.begin(115200);
    while (!ir) {
        delay(10);
    }
    delay(1000);
}

void loop1() {
    while (!start_flag) {
        delay(10);
    }
    ir.write(255);
    if (ir.available() > 5) {
        int recv_data = ir.read();
        if (recv_data == 255) {
            byte data[4];
            data[0] = ir.read();
            data[1] = ir.read();
            data[2] = ir.read();
            data[3] = ir.read();
            float _ir_angle = (float)(data[0] + (data[1] << 8)) / 100.0 - PI;
            if (abs(_ir_angle) <= PI) {
                ir_angle = _ir_angle;
            }
            ir_radius = (float)data[2];
            ball_flag = data[3];
        }
    }
    float A = pow(B, ir_radius);
    move_angle = ir_angle * A;
    move_angle += ir_angle;
    float vx = sin(move_angle);
    float vy = cos(move_angle);
    vx = (vx + 1.0) * 100.0;
    vy = (vy + 1.0) * 100.0;

    Serial.print("ir_angle: ");
    Serial.print(ir_angle);
    Serial.print("\tir_radius: ");
    Serial.print(ir_radius);
    Serial.print("\tmove_angle: ");
    Serial.println(move_angle);

    byte data[8];
    data[0] = vx;  // 送るもとの値は -1~1 だが、送るときは 0~200 にする
    data[1] = vy;
    data[2] = 100;
    data[3] = (byte)((gyro.angle + PI) * 100);
    data[4] = (byte)((int)((gyro.angle + PI) * 100) >> 8);
    data[5] = (byte)((machine_angle + PI) * 100);
    data[6] = (byte)((int)((machine_angle + PI) * 100) >> 8);
    data[7] = 0;

    motor.write(255);
    motor.write(data, 8);
    delay(10);
}*/
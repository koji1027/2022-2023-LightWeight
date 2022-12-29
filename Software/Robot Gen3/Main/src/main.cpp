#include <Arduino.h>

#include "MPU6050/gyro.h"
#include "led.h"
#include "line.h"

SerialPIO motor(22, 16, 32);
SerialPIO ir(1, 0, 32);
Gyro gyro;
Line line;

float machine_angle = 0.0;
float ir_angle = 0.0;
float ir_radius = 0.0;
int ball_flag = 0;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    motor.begin(115200);
    ir.begin(115200);

    gyro.begin();
    line.begin();
    delay(1000);
}

void loop() {
    // put your main code here, to run repeatedly:
    gyro.getEuler();

    ir.write(255);
    while (ir.available() < 5) {
    }
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

    byte data[8];
    float vx = ir_radius * sin(ir_angle);
    vx = constrain(vx, -120, 120);
    vx = pow(vx, 3.0) * 5.787037037037037e-7;
    vx = (vx + 1) * 100.0;
    float vy = 0 * 100.0;
    vy += 100;
    data[0] = vx;  // 送るもとの値は -1~1 だが、送るときは 0~200 にする
    data[1] = vy;
    data[2] = 100;
    data[3] = (byte)((gyro.angle + PI) * 100);
    data[4] = (byte)((int)((gyro.angle + PI) * 100) >> 8);
    data[5] = (byte)((machine_angle + PI) * 100);
    data[6] = (byte)((int)((machine_angle + PI) * 100) >> 8);
    data[7] = 0;

    Serial.print("Vx: ");
    Serial.print(data[0]);
    Serial.print("\tVy: ");
    Serial.println(data[1]);

    motor.write(255);
    motor.write(data, 8);
    delay(10);
}
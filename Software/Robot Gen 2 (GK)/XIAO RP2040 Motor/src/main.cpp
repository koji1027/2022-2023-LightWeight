#include <Arduino.h>

#include "motor.h"

Motor motor;

bool start_flag = false;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    motor.begin();
    motor.stop();
    while (!start_flag) {
    }
    delay(10);
}

void loop() {
    while (start_flag) {
        if (motor.distance == 255) {
            motor.distance = 70;
        }
        motor.c = 0;
        if (motor.c == 0) {
            // ライン踏んでないとき
            /*if (motor.distance > 72) {
                motor.move_angle = -PI;
                int diff = abs(motor.distance - 70);
                motor.speed = diff * GK_Kp;
                motor.cal();
                Serial.print("down : ");
                Serial.println(motor.distance);
            } else if (motor.distance < 68) {
                motor.move_angle = 0;
                int diff = 70 - motor.distance;
                motor.speed = abs(diff * GK_Kp);
                motor.cal();
                Serial.print("up : ");
                Serial.println(motor.distance);
            } else {
                if (motor.move_angle > 0) {
                    motor.move_angle = PI / 2.0;
                } else if (motor.move_angle < 0) {
                    motor.move_angle = -PI / 2.0;
                }
            }*/
            /*if (motor.distance < 85) {
                motor.speed = 100;
                motor.cal();
                Serial.println(motor.distance);
            } else {
                if (motor.move_angle > 0) {
                    motor.move_angle = PI / 2.0;
                } else if (motor.move_angle < 0) {
                    motor.move_angle = -PI / 2.0;
                }
                motor.speed = 100;
                motor.cal();
                delay(100);
            }*/
            if (motor.distance < 75 && motor.distance > 65) {
                motor.speed = 100;
                if (motor.move_angle > 0) {
                    motor.move_angle = PI / 2.0;
                    motor.speed = 0;
                    motor.cal();
                } else if (motor.move_angle < 0) {
                    motor.move_angle = -PI / 2.0;
                    motor.speed = 0;
                    motor.cal();
                }
            } else if (motor.distance <= 65) {
                while (motor.c == 0 && motor.distance <= 65) {
                    motor.move_angle = 0;
                    motor.speed = 100;
                    motor.cal();
                }
            } else if (motor.c == 0 && motor.distance >= 75) {
                while (motor.distance >= 75) {
                    motor.move_angle = -PI;
                    motor.speed = 100;
                    motor.cal();
                }
            }
        } else {  // ライン踏んだとき
            motor.speed = 100;
            motor.cal();
            delay(50);
        }
    }
}

void setup1() { Serial1.begin(115200); }

void loop1() {
    if (Serial1.available() > 6) {
        if (!start_flag) {
            start_flag = true;
        }
        int recv_data = Serial1.read();
        if (recv_data == 255) {
            int data[6];  //[0][1]:gyro, [2][3]:move
            data[0] = Serial1.read();
            data[1] = Serial1.read();
            data[2] = Serial1.read();
            data[3] = Serial1.read();
            data[4] = Serial1.read();
            data[5] = Serial1.read();
            int recv_gyro = data[0] + (data[1] << 8);
            motor.gyro_angle = (recv_gyro / 100.0) - PI;
            int recv_move = data[2] + (data[3] << 8);
            motor.c = data[4];  // 1:line, 0:move
            if (motor.c == 1) {
                motor.move_angle = recv_move / 100.0;
                if (motor.move_angle > PI) {
                    motor.move_angle -= 2 * PI;
                }
            } else {
                motor.move_angle = (recv_move / 100.0) - PI;
            }
            motor.distance = data[5];
        }
    }
}
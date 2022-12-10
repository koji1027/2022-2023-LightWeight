#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

#define PWM_FREQ 500000
#define Kp 25.0
#define Kd 1.0
#define Ki 10.0

class Motor {
   public:
    float gyro_angle = 0.0;
    float go_angle = 0.0;
    int speed = 0;
    void begin();
    void cal();
    void move(float *power);
    void stop();

   private:
    const int MOTOR[4][2] = {{D0, D1}, {D2, D3}, {D4, D5}, {D8, D9}};
    const float MOTOR_POS[4] = {3.0 * PI / 4.0, PI / 4.0, 7.0 * PI / 4.0,
                                5.0 * PI / 4.0};
    unsigned long long pre_time = 0;
    float pre_gyro_angle = 0.0;
    float I = 0.0;
};

void Motor::begin() {
    analogWriteFreq(PWM_FREQ);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            pinMode(MOTOR[i][j], OUTPUT);
        }
        digitalWrite(MOTOR[i][0], HIGH);
        analogWrite(MOTOR[i][1], 128);
    }
}

void Motor::cal() {
    float power[4] = {0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < 4; i++) {
        power[i] = -sin(go_angle - MOTOR_POS[i]) * speed;
    }
    int max_power = 0;
    for (int i = 0; i < 4; i++) {
        if (abs(power[i]) > max_power) {
            max_power = abs(power[i]);
        }
    }
    if (max_power != 0) {
        for (int i = 0; i < 4; i++) {
            power[i] = power[i] / max_power * speed;
        }
    }
    float PID, P, D = 0.0;
    unsigned long long now = micros();
    unsigned long long dt = (now - pre_time);
    P = gyro_angle * Kp;
    D = (gyro_angle - pre_gyro_angle) / dt * Kd * 1000000.0;
    if (abs(gyro_angle) < radians(4)) {
        I = 0.0;
    }
    I += gyro_angle * dt * Ki / 1000000.0;
    PID = P + I + D;
    pre_time = now;
    pre_gyro_angle = gyro_angle;
    for (int i = 0; i < 4; i++) {
        power[i] -= PID;
        constrain(power[i], -100, 100);
    }
    move(power);
}

void Motor::move(float *power) {
    for (int i = 0; i < 4; i++) {
        power[i] += 128;
        digitalWrite(MOTOR[i][0], HIGH);
        analogWrite(MOTOR[i][1], power[i]);
    }
}

void Motor::stop() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(MOTOR[i][0], LOW);
        analogWrite(MOTOR[i][1], 128);
    }
}

#endif
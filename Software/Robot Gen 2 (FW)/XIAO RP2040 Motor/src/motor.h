#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

#define PWM_FREQ 100000
#define Kp 1.8
#define Kd 0.00005
#define Ki 0
#define LPF 0.2

class Motor {
   public:
    float gyro_angle = 0.0;
    float move_angle = 0.0;
    float machine_angle = 0.0;
    int default_speed = 100;
    int speed;
    int c = 0;  // 1:line, 0:move
    void begin();
    void cal();
    void move(float power[4]);
    void stop();

   private:
    const int MOTOR_PIN[4][2] = {{D2, D3}, {D0, D1}, {D8, D9}, {D4, D5}};
    const float MOTOR_POS[4][2] = {
        {-1.0, 1.0}, {1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}};
    unsigned long long pre_time = 0;
    float pre_diff = 0.0;
    float I = 0.0;
    float pre_power[4] = {0.0, 0.0, 0.0, 0.0};
};

void Motor::begin() {
    analogWriteFreq(PWM_FREQ);
    analogWriteResolution(9);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            pinMode(MOTOR_PIN[i][j], OUTPUT);
        }
        digitalWrite(MOTOR_PIN[i][0], HIGH);
        analogWrite(MOTOR_PIN[i][1], 256);
    }
}

void Motor::cal() {
    float power[4] = {0.0, -1.0, -1.0, -1.0};
    if (speed != 0) {

        float vx = speed * sin(move_angle);
        float vy = speed * cos(move_angle);
        for (int i = 0; i < 4; i++) {
            power[i] = vx * MOTOR_POS[i][0] + vy * MOTOR_POS[i][1];
        }
        float max_power = 0.0;
        for (int i = 0; i < 4; i++) {
            if (abs(power[i]) > max_power) {
                max_power = abs(power[i]);
            }
        }
        if (max_power > 100.0) {
            for (int i = 0; i < 4; i++) {
                power[i] = power[i] * speed / max_power;
            }
        }
    }
    float P, D = 0.0;
    float diff = machine_angle - gyro_angle;
    diff = diff / PI * 180.0;
    unsigned long long now_time = micros();
    float dt = (float)(now_time - pre_time) / 1000000.0;
    pre_time = now_time;
    if (abs(diff) < 3.0) {
        I = 0.0;
    }
    P = Kp * diff;
    D = Kd * (diff - pre_diff) / dt;
    I = I + Ki * diff * dt;
    pre_diff = diff;
    float PID = P + D + I;
    PID *= -1;
    for (int i = 0; i < 4; i++) {
        power[i] += PID;
        power[i] = constrain(power[i], -200.0, 200.0);
    }
    for (int i = 0; i < 4; i++) {
        power[i] = pre_power[i] * LPF + power[i] * (1.0 - LPF);
        pre_power[i] = power[i];
    }
    move(power);
}

void Motor::move(float power[4]) {
    for (int i = 0; i < 4; i++) {
        power[i] += 256;
        digitalWrite(MOTOR_PIN[i][0], HIGH);
        analogWrite(MOTOR_PIN[i][1], (int)power[i]);
    }
}

void Motor::stop() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(MOTOR_PIN[i][0], LOW);
        analogWrite(MOTOR_PIN[i][1], 256);
    }
}

#endif
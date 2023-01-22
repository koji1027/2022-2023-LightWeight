#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

#define PWM_FREQ 100000
#define Kp 1.8
#define Kd 0.00005
#define Ki 0
#define LPF 0.2
#define MAX_SPEED 255

class Motor
{
public:
    void begin();
    void cal(float vx, float vy, int speed, float machine_angle, float gyro_angle);
    void move(float power[4]);
    void brake();

private:
    const int MOTOR_PIN[4][2] = {{D2, D3}, {D0, D1}, {D8, D9}, {D4, D5}};
    const float MOTOR_POS[4][2] = {
        {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}};
    unsigned long long pre_time = 0;
    float pre_diff = 0.0;
    float I = 0.0;
    float pre_power[4] = {0.0, 0.0, 0.0, 0.0};
};

#endif
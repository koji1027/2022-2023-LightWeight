#include <Arduino.h>

#define MOTOR_NUM 4
#define PWM_RES 9
#define MAX_PWM pow(2, PWM_RES)
#define PWM_FREQ 100000
#define KP 1.8
#define KI 0.00005
#define KD 0.0

class Motor
{
public:
        void begin(void);
        void cal(double move_angle, double gyro_angle, uint8_t speed);
        void drive(int *p);
        void brake(void);
        void release(void);

private:
        const int PWM_PIN[MOTOR_NUM] = {D2, D0, D8, D4}; // LAP駆動なのでHIGHに固定
        const int DIR_PIN[MOTOR_NUM] = {D3, D1, D9, D5}; // LAP駆動なのでDIRでPWM出力
        const double MOTOR_RAD[MOTOR_NUM] = {PI / 4.0, PI * 3.0 / 4.0, PI * 5.0 / 4.0, PI * 7.0 / 4.0};
        double SIN[628];
        unsigned long long pre_time = 0;
};

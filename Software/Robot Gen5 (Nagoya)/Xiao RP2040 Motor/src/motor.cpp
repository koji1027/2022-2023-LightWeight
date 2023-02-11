#include "motor.h"

void Motor::begin(void)
{
        analogWriteResolution(PWM_RES);
        analogWriteFreq(PWM_FREQ);
        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
                pinMode(PWM_PIN[i], OUTPUT);
                digitalWrite(PWM_PIN[i], HIGH);
                pinMode(DIR_PIN[i], OUTPUT);
                analogWrite(DIR_PIN[i], MAX_PWM / 2);
        }
}

void Motor::drive(int *p)
{
        uint16_t duty[MOTOR_NUM];
        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
                duty[i] = 256 - p[i];
                digitalWrite(PWM_PIN[i], HIGH);
                analogWrite(DIR_PIN[i], duty[i]);
        }
}

void Motor::brake(void)
{
        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
                digitalWrite(PWM_PIN[i], LOW);
        }
}

void Motor::release(void)
{
        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
                digitalWrite(PWM_PIN[i], HIGH);
                analogWrite(DIR_PIN[i], MAX_PWM / 2);
        }
}

void Motor::cal(double move_angle, double gyro_angle, uint8_t speed)
{
        double power_ratio[MOTOR_NUM];
        double gyro_angle_diff = 0 - gyro_angle;
        unsigned long long now_time = micros();
        unsigned long long dt = now_time - pre_time;
        double p, i, d;
        p = gyro_angle_diff * KP;
        i = gyro_angle_diff * dt * KI / 1000000.0;
        d = gyro_angle_diff * 1000000.0 / dt * KD;
        double pid = (p + i + d) / PI * 180.0;
        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
                power_ratio[i] = sin(move_angle - MOTOR_RAD[i]);
        }
        double max = 0;
        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
                if (abs(power_ratio[i]) > max)
                {
                        max = abs(power_ratio[i]);
                }
        }
        if (max > 0)
        {
                for (uint8_t i = 0; i < MOTOR_NUM; i++)
                {
                        power_ratio[i] *= speed / max;
                }
        }
        int power[MOTOR_NUM];
        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
                power_ratio[i] += pid;
                power_ratio[i] = constrain(power_ratio[i], -255, 255);
                power[i] = (int)power_ratio[i];
        }
        drive(power);
}
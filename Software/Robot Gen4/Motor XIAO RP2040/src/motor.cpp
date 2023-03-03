#include <motor.h>

void Motor::begin()
{
    analogWriteFreq(PWM_FREQ);
    analogWriteResolution(9);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            pinMode(MOTOR_PIN[i][j], OUTPUT);
        }
        digitalWrite(MOTOR_PIN[i][0], HIGH);
        analogWrite(MOTOR_PIN[i][1], 256);
    }
}

void Motor::cal(float vx, float vy, int speed, float machine_angle,
                float gyro_angle)
{
    float power[4] = {0.0, 0.0, 0.0, 0.0};
    float diff = (gyro_angle - machine_angle) / PI * 180.0;
    
    if (abs(diff) < 50)
    {
        if (vx != 0.0 || vy != 0.0)
        {
            for (int i = 0; i < 4; i++)
            {
                power[i] =
                    (vx * MOTOR_POS[i][0] + vy * MOTOR_POS[i][1]) * speed;
            }
        }
    }
    unsigned long long now_time = micros();
    float dt = (now_time - pre_time) / 1000000.0;
    float PID;
    float P = diff * Kp;
    I += diff * dt * Ki;
    float D = (diff - pre_diff) / dt * Kd;
    pre_diff = diff;
    pre_time = now_time;
    PID = P + I + D;
    PID = constrain(PID, -200, 200);
    for (int i = 0; i < 4; i++)
    {
        power[i] += PID;
        power[i] = constrain(power[i], -200, 200);
    }
    for (int i = 0; i < 4; i++)
    {
        power[i] = power[i] * LPF + pre_power[i] * (1 - LPF);
        pre_power[i] = power[i];
        pre_power[i] = constrain(pre_power[i], -200, 200);
    }
    //Serial.println(power[1]);
    move(power);
}

void Motor::move(float power[4])
{
    for (int i = 0; i < 4; i++)
    {
        Serial.print(power[i]);
        Serial.print("\t");
        power[i] += 256;
        digitalWrite(MOTOR_PIN[i][0], HIGH);
        analogWrite(MOTOR_PIN[i][1], (int)power[i]);
    }
    Serial.println();
}

void Motor::brake()
{
    for (int i = 0; i < 4; i++)
    {
        digitalWrite(MOTOR_PIN[i][0], LOW);
        analogWrite(MOTOR_PIN[i][1], 256);
    }
}
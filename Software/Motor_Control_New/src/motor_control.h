#include <Arduino.h>

#define MOTOR_NUM 4

class motor_control
{
public:
    void begin();
    void cal(float vel_x, float vel_y, float vel_theta, int max_speed);
    void move(float power[MOTOR_NUM]);

private:
    const int MOTOR_PIN[MOTOR_NUM][2] = {{3, 2}, {10, 9}, {12, 11}, {18, 13}};
    const int MOTOR_POS[MOTOR_NUM] = {240, 300, 60, 120};
    float COS[360];
    float SIN[360];
};

void motor_control::begin()
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        pinMode(MOTOR_PIN[i][0], OUTPUT);
        pinMode(MOTOR_PIN[i][1], OUTPUT);
        digitalWriteFast(MOTOR_PIN[i][0], LOW);
        digitalWriteFast(MOTOR_PIN[i][1], LOW);
    }
    for (int i = 0; i < 360; i++)
    {
        COS[i] = cos(i * PI / 180);
        SIN[i] = sin(i * PI / 180);
    }
}

void motor_control::cal(float vel_x, float vel_y, float vel_theta, int speed)
{
    float power[MOTOR_NUM];
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        power[i] = vel_x * COS[MOTOR_POS[i]] + vel_y * SIN[MOTOR_POS[i]] + vel_theta;
    }
    float max_power = 0;
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        if (abs(power[i]) > max_power)
        {
            max_power = abs(power[i]);
        }
    }
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        power[i] = power[i] / max_power * speed;
    }
    move(power);
}

void motor_control::move(float power[MOTOR_NUM])
{
    /*Serial.print("power: ");
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        Serial.print(power[i]);
        Serial.print(" ");
    }
    Serial.println();*/
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        int dir = 0;
        if (power[i] < 0)
        {
            dir = 1;
            power[i] = -power[i];
        }
        analogWrite(MOTOR_PIN[i][0], round(power[i]));
        digitalWriteFast(MOTOR_PIN[i][1], dir);
    }
}
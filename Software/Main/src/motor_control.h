#include <Arduino.h>

#define MOTOR_NUM 4
#define MAX_PWM 255
#define PROPORTIONAL_GAIN 30

class Motor
{
public:
    void init();
    void cal_pwm(float dir, int power);
    void correct_pos(float target_pos, float current_pos);
    void move();

private:
    int get_max_index(float data[], int data_num);
    const int MOTOR_PWM[MOTOR_NUM] = {1, 5, 9, 13};
    const int MOTOR_DIR[MOTOR_NUM] = {2, 6, 10, 14};
    const float MOTOR_POS[MOTOR_NUM] = {PI / 3.0, 2 * PI / 3.0, 4 * PI / 3.0, 5 * PI / 3.0};
    int pwm[MOTOR_NUM];
};

void Motor::init()
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        pinMode(MOTOR_PWM[i], OUTPUT);
        pinMode(MOTOR_DIR[i], OUTPUT);
    }
}

void Motor::cal_pwm(float dir, int power)
{
    float _pwm[MOTOR_NUM];
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        _pwm[i] = sin(dir - MOTOR_POS[i]) * float(power);
    }
    int max_index = get_max_index(_pwm, MOTOR_NUM);
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        pwm[i] = round(_pwm[i] / _pwm[max_index] * float(power));
    }
}

void Motor::correct_pos(float target_pos, float current_pos)
{
    float diff = target_pos - current_pos;
    int adjust_power = round(diff * PROPORTIONAL_GAIN);
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        pwm[i] += adjust_power;
    }
}

void Motor::move()
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        if (pwm[i] > 0)
        {
            digitalWrite(MOTOR_DIR[i], HIGH);
        }
        else
        {
            digitalWrite(MOTOR_DIR[i], LOW);
        }
        analogWrite(MOTOR_PWM[i], abs(pwm[i]));
    }
}

int Motor::get_max_index(float data[], int data_num)
{
    int max_index = 0;
    int max = 0;
    for (int i = 0; i < data_num; i++)
    {
        if (data[i] > max)
        {
            max = data[i];
            max_index = i;
        }
    }
    return max_index;
}
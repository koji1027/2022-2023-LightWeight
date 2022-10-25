#include <Arduino.h>

#define MOTOR_NUM 4
#define Kp 1.8
#define Ki 0
#define Kd 0.17
#define DELTA_TIME 0.01
#define MOTOR_POWER 200

class motor_control
{
public:
    void begin();
    void cal(float ir_deg, int speed, float target_deg, float current_deg);
    void move(float power[MOTOR_NUM]);
    void break_all();
    // void posture_spin(float gyro_degree);

private:
    const int MOTOR_PIN[MOTOR_NUM][2] = {{3, 2}, {10, 9}, {12, 11}, {13, 18}};
    const int MOTOR_POS[MOTOR_NUM] = {60, 120, 240, 300};
    float COS[360];
    float SIN[360];
    float dt, preTime;
    float P, I, D;
    float deg_diff[2] = {0, 0};
};

void motor_control::begin()
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        pinMode(MOTOR_PIN[i][0], OUTPUT);
        pinMode(MOTOR_PIN[i][1], OUTPUT);
        analogWriteResolution(9);
        analogWriteFrequency(MOTOR_PIN[i][1], 80000);
        digitalWriteFast(MOTOR_PIN[i][0], HIGH);
        analogWrite(MOTOR_PIN[i][1], 255);
    }
    for (int i = 0; i < 360; i++)
    {
        COS[i] = cos(i * PI / 180);
        SIN[i] = sin(i * PI / 180);
    }
}

void motor_control::cal(float ir_deg, int speed, float target_deg, float current_deg)
{
    float power[MOTOR_NUM];
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        float deg = ir_deg - MOTOR_POS[i];
        float rad = deg * PI / 180.0;
        power[i] = sin(rad) * speed;
    }
    float max_power = 0;
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        if (abs(power[i]) > max_power)
        {
            max_power = abs(power[i]);
        }
    }
    if (max_power != 0)
    {
        for (int i = 0; i < MOTOR_NUM; i++)
        {
            power[i] = power[i] / max_power * speed;
        }
    }

    dt = (micros() - preTime) / 1000000;
    deg_diff[1] = target_deg - current_deg;
    P = Kp * deg_diff[1];
    I += Ki * deg_diff[1] * dt;
    D = Kd * (deg_diff[1] - deg_diff[0]) / dt;
    deg_diff[0] = deg_diff[1];
    float vel_theta = P + I + D;
    preTime = micros();

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        power[i] -= vel_theta;
    }
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        power[i] = constrain(power[i], -255, 255);
    }
    // Serial.print("current_deg: ");
    // Serial.print(current_deg);
    /*Serial.print(" power: ");
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        Serial.print(power[i]);
        Serial.print(" ");
    }*/
    move(power);
}

void motor_control::move(float power[MOTOR_NUM])
{
    // Serial.print("power : ");
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        power[i] += 256;
        power[i] = 511 - power[i];
        // Serial.print(power[i]);
        // Serial.print(", ");
        pinMode(MOTOR_PIN[i][1], OUTPUT);
        digitalWriteFast(MOTOR_PIN[i][0], HIGH);
        analogWrite(MOTOR_PIN[i][1], (int)power[i]);
    }
    // Serial.println();
}

void motor_control::break_all()
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        digitalWriteFast(MOTOR_PIN[i][0], LOW);
    }
}

/*void motor_control::posture_spin(float gyro_degree)
{
    dir_diff[1] = gyro_degree;
    P = dir_diff[1] * Kp;
    I = I + ((dir_diff[1] + dir_diff[0]) / 2) * DELTA_TIME * Ki;
    D = (dir_diff[1] - dir_diff[0]) / DELTA_TIME * Kd;
    float spin_power = P + I + D;
    if (spin_power >= 0)
    {
        for (int i = 0; i < MOTOR_NUM; i++)
        {
            analogWrite(MOTOR_PIN[i][0], round(abs(spin_power)));
            digitalWriteFast(MOTOR_PIN[i][1], 0);
        }
    }
    else
    {
        for (int i = 0; i < MOTOR_NUM; i++)
        {
            analogWrite(MOTOR_PIN[i][0], round(abs(spin_power)));
            digitalWriteFast(MOTOR_PIN[i][1], 1);
        }
    }
    // Serial.println(round(abs(spin_power)));
    dir_diff[0] = dir_diff[1];
}*/
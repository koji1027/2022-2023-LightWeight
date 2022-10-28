#include <Arduino.h>

#define MOTOR_NUM 4
#define Kp 2.2
#define Ki 0.1
#define Kd 0.09
#define DELTA_TIME 0.01
#define MOTOR_POWER 255
#define SQRT3 1.7320508075688772935274463415059

class motor_control
{
public:
    void begin();
    void cal(float ir_deg, int speed, float target_deg, float current_deg);
    void move(float power[MOTOR_NUM]);
    void stop();
    // void posture_spin(float gyro_degree);

private:
    const int MOTOR_PIN[MOTOR_NUM][2] = {{3, 2}, {10, 9}, {12, 11}, {13, 18}};
    const float MOTOR_POS[MOTOR_NUM][2] = {{-1,SQRT3}, {1, SQRT3}, {1, -SQRT3}, {-1, -SQRT3}};
    float dt, preTime;
    float P, I = 0, D;
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
}

void motor_control::cal(float ir_deg, int speed, float target_deg, float current_deg)
{
    float power[MOTOR_NUM] = {0, 0, 0, 0};
    float vx = sin(radians(ir_deg));
    float vy = cos(radians(ir_deg));
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        power[i] = vx * MOTOR_POS[i][0] + vy * MOTOR_POS[i][1];
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
    power[0] -= 2;
    power[1] -= 2;
    power[2] -= 2;
    power[3] -= 1;
    power[0] = constrain(power[0], -255, 255);
    power[1] = constrain(power[1], -255, 255);
    power[2] = constrain(power[2], -255, 255);
    power[3] = constrain(power[3], -255, 255);
    /*if (current_deg < 3 && current_deg > -3) {
        I = 0;  //要検討
    }*/
    dt = (micros() - preTime) / 1000000;
    deg_diff[1] = target_deg - current_deg;
    P = Kp * deg_diff[1];
    I += Ki * deg_diff[1] * dt;
    D = Kd * (deg_diff[1] - deg_diff[0]) / dt;
    deg_diff[0] = deg_diff[1];
    float vel_theta = P + I + D;
    preTime = micros();
    vel_theta = constrain(vel_theta, -200,200);
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        power[i] -= vel_theta;
    }
    if ((ir_deg > 70 && ir_deg < 110) || (ir_deg < -70 && ir_deg > -110))
    {
        power[0] -= 0;
        power[1] -= 0;
        power[2] -= 0;
        power[3] -= 0;
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
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        power[i] += 256;
        pinMode(MOTOR_PIN[i][1], OUTPUT);
        digitalWriteFast(MOTOR_PIN[i][0], HIGH);
        analogWrite(MOTOR_PIN[i][1], (int)power[i]);
    }
    //Serial.println();
}

void motor_control::stop()
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        digitalWriteFast(MOTOR_PIN[i][1], LOW);
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
#include <Arduino.h>

#define MOTOR_NUM 4
#define MAX_PWM 800
#define PWM_FREQ 80000
#define Kp 0.0
#define Ki 0.0
#define Kd 0.0

class Motor {
   public:
    void begin();
    void brake();
    void cal(float go_rad, float gyro_rad, int speed, float machine_rad);
    void move(float *power);

   private:
    const int MOTOR_PIN[MOTOR_NUM][2] = {
        {0, 1}, {2, 3}, {4, 5}, {8, 9}};  // PWM, DIR
    const float MOTOR_POS[MOTOR_NUM] = {0.0, PI / 2.0, PI, 3.0 * PI / 2.0};
    float power[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0};
};

void Motor::begin() {
    for (int i = 0; i < MOTOR_NUM; i++) {
        pinMode(MOTOR_PIN[i][0], OUTPUT);
        pinMode(MOTOR_PIN[i][1], OUTPUT);
        digitalWrite(MOTOR_PIN[i][0], HIGH);
        pwm(MOTOR_PIN[i][1], PWM_FREQ, 1023.0 / 2.0);
    }
}

void Motor::brake() {
    for (int i = 0; i < MOTOR_NUM; i++) {
        digitalWrite(MOTOR_PIN[i][0], LOW);
        digitalWrite(MOTOR_PIN[i][1], LOW);
    }
}

void Motor::cal(float go_rad, float gyro_rad, int speed, float machine_rad) {
    go_rad = fmod(go_rad, 2.0 * PI);
    gyro_rad = fmod(gyro_rad, 2.0 * PI);
    machine_rad = fmod(machine_rad, 2.0 * PI);
    speed = constrain(speed, -MAX_PWM / 2.0, MAX_PWM / 2.0);
    for (int i = 0; i < MOTOR_NUM; i++) {
        power[i] = sin(go_rad - MOTOR_POS[i]);
    }
    float max_power = 0.0;
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (max_power < abs(power[i])) {
            max_power = abs(power[i]);
        }
    }
    for (int i = 0; i < MOTOR_NUM; i++) {
        power[i] = power[i] / max_power * speed;
    }
    move(power);
}

void Motor::move(float *power) {
    int output[MOTOR_NUM] = {0};
    for (int i = 0; i < MOTOR_NUM; i++) {
        output[i] = (int)(power[i] + 1023.0 / 2.0);
        output[i] = constrain(output[i], 0, MAX_PWM);
    }
    for (int i = 0; i < MOTOR_NUM; i++) {
        digitalWrite(MOTOR_PIN[i][0], HIGH);
        pwm(MOTOR_PIN[i][1], PWM_FREQ, output[i]);
    }
}
#include <Arduino.h>

#define MOTOR_NUM 4
#define SQRT3 1.73205080756887729352744634150587236694280525381038062805580
#define Kp 1.8
#define Ki 150
#define Kd 0.00005
#define MAX_PWM 200

class motor {
   private:
    const int MOTOR_PIN[MOTOR_NUM][2] = {{10, 9}, {3, 2}, {13, 18}, {12, 11}};
    const float MOTOR_POS[MOTOR_NUM][2] = {
        {1, -SQRT3}, {-1, -SQRT3}, {-1, SQRT3}, {1, SQRT3}};
    float I = 0;
    float pre_diff = 0;
    unsigned long long pre_time = 0;

   public:
    void init();
    void cal(float dir, int duty, float target_dir, float current_dir);
    void move(float power[MOTOR_NUM]);
    void stop();
};

void motor::init() {
    for (int i = 0; i < MOTOR_NUM; i++) {
        pinMode(MOTOR_PIN[i][0], OUTPUT);
        pinMode(MOTOR_PIN[i][1], OUTPUT);
        analogWriteResolution(9);
        analogWriteFrequency(MOTOR_PIN[i][1], 80000);
        digitalWriteFast(MOTOR_PIN[i][0], HIGH);
        analogWrite(MOTOR_PIN[i][1], 255);
    }
}

void motor::cal(float dir, int duty, float target_dir, float current_dir) {
    dir = fmod(dir, PI * 2.0) - PI;
    duty = constrain(duty, 0, MAX_PWM);
    target_dir = fmod(target_dir, PI * 2.0) - PI;
    current_dir = fmod(current_dir, PI * 2.0) - PI;
    float power[MOTOR_NUM] = {0};
    if (duty != 0) {
        float vx = duty * sin(dir);
        float vy = duty * cos(dir);
        for (int i = 0; i < MOTOR_NUM; i++) {
            power[i] = vx * MOTOR_POS[i][0] + vy * MOTOR_POS[i][1];
        }
    }
    if (duty != 0) {
        float max_power = 0;
        for (int i = 0; i < MOTOR_NUM; i++) {
            max_power = max(max_power, abs(power[i]));
        }
        for (int i = 0; i < MOTOR_NUM; i++) {
            power[i] = power[i] / max_power * duty;
        }
    }
    power[0] *= 1.0;
    power[1] *= 0.65;
    power[2] *= 0.65;
    power[3] *= 1.0;
    float P, D = 0;
    float diff = target_dir - current_dir;
    diff *= 180.0 / PI;
    unsigned long long now_time = millis();
    float dt = (float)(now_time - pre_time) / 1000000.0;
    pre_time = now_time;
    if (abs(diff) < 3.0) {
        I = 0;
    }
    P = Kp * diff;
    I += Ki * diff * dt;
    D = (diff - pre_diff) / dt * Kd;
    pre_diff = diff;
    float pid = P + I + D;
    for (int i = 0; i < MOTOR_NUM; i++) {
        power[i] -= pid;
        power[i] = constrain(power[i], -MAX_PWM, MAX_PWM);
    }
    move(power);
}

void motor::move(float power[MOTOR_NUM]) {
    for (int i = 0; i < MOTOR_NUM; i++) {
        int duty = power[i] + 256;
        duty = constrain(duty, 1, 511);
        digitalWriteFast(MOTOR_PIN[i][0], HIGH);
        analogWrite(MOTOR_PIN[i][1], duty);
    }
}

void motor::stop() {
    for (int i = 0; i < MOTOR_NUM; i++) {
        digitalWriteFast(MOTOR_PIN[i][0], LOW);
        analogWrite(MOTOR_PIN[i][1], 255);
    }
}
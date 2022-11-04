#include <Arduino.h>

#define MOTOR_NUM 4
#define Kp 2.4
#define Ki 0.1
#define Kd 0.15
#define DELTA_TIME 0.01
#define MOTOR_POWER 255
#define SQRT3 1.7320508075688772935274463415059

class motor_control {
   public:
    void begin();
    void cal(float ir_deg, int speed, float target_deg, float current_deg);
    void move(float power[MOTOR_NUM]);
    void stop();
    // void posture_spin(float gyro_degree);

   private:
    const int MOTOR_PIN[MOTOR_NUM][2] = {{3, 2}, {10, 9}, {12, 11}, {13, 18}};
    const float MOTOR_POS[MOTOR_NUM][2] = {
        {-1, SQRT3}, {1, SQRT3}, {1, -SQRT3}, {-1, -SQRT3}};
    float dt, preTime;
    float P, I = 0, D;
    float deg_diff[2] = {0, 0};
};

void motor_control::begin() {
    for (int i = 0; i < MOTOR_NUM; i++) {
        pinMode(MOTOR_PIN[i][0], OUTPUT);
        pinMode(MOTOR_PIN[i][1], OUTPUT);
        analogWriteResolution(9);
        analogWriteFrequency(MOTOR_PIN[i][1], 80000);
        digitalWriteFast(MOTOR_PIN[i][0], HIGH);
        analogWrite(MOTOR_PIN[i][1], 255);
    }
}

void motor_control::cal(float ir_deg, int speed, float target_deg,
                        float current_deg) {
    ir_deg = fmod(ir_deg, 360);
    ir_deg = ir_deg > 180 ? ir_deg - 360 : ir_deg;
    target_deg = fmod(target_deg, 360);
    target_deg = target_deg > 180 ? target_deg - 360 : target_deg;
    current_deg = fmod(current_deg, 360);
    current_deg = current_deg > 180 ? current_deg - 360 : current_deg;
    /*if (ir_deg > 60 && ir_deg < 120) {
        target_deg = ir_deg;
    } else if (ir_deg < -60 && ir_deg > -120) {
        target_deg = ir_deg;
    }*/
    float power[MOTOR_NUM] = {0, 0, 0, 0};
    float vx = sin(radians(ir_deg - target_deg));
    float vy = cos(radians(ir_deg - target_deg));
    for (int i = 0; i < MOTOR_NUM; i++) {
        power[i] = vx * MOTOR_POS[i][0] + vy * MOTOR_POS[i][1];
    }
    float max_power = 0;
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (abs(power[i]) > max_power) {
            max_power = abs(power[i]);
        }
    }
    if (max_power != 0) {
        for (int i = 0; i < MOTOR_NUM; i++) {
            power[i] = power[i] / max_power * speed;
        }
    }
    power[0] -= 2;
    power[1] -= 2;
    power[2] -= 2;
    power[3] -= 1;
    power[0] = constrain(power[0], -200, 200);
    power[1] = constrain(power[1], -200, 200);
    power[2] = constrain(power[2], -200, 200);
    power[3] = constrain(power[3], -200, 200);

    if (current_deg < 3 && current_deg > -3) {
        I = 0;
    }
    dt = (micros() - preTime) / 1000000;
    deg_diff[1] = target_deg - current_deg;
    //Serial.println(deg_diff[1]);
    deg_diff[1] = fmod(deg_diff[1], 360);
    deg_diff[1] = deg_diff[1] > 180 ? deg_diff[1] - 360 : deg_diff[1];
    P = Kp * deg_diff[1];
    I += Ki * deg_diff[1] * dt;
    D = Kd * (deg_diff[1] - deg_diff[0]) / dt;
    deg_diff[0] = deg_diff[1];
    float vel_theta = P + I + D;
    preTime = micros();
    vel_theta = constrain(vel_theta, -200, 200);
    for (int i = 0; i < MOTOR_NUM; i++) {
        power[i] -= vel_theta;
    }
    for (int i = 0; i < MOTOR_NUM; i++) {
        power[i] = constrain(power[i], -200, 200);
    }
    move(power);
}

void motor_control::move(float power[MOTOR_NUM]) {
    for (int i = 0; i < MOTOR_NUM; i++) {
        power[i] += 256;
        pinMode(MOTOR_PIN[i][1], OUTPUT);
        digitalWriteFast(MOTOR_PIN[i][0], HIGH);
        analogWrite(MOTOR_PIN[i][1], (int)power[i]);
    }
}

void motor_control::stop() {
    for (int i = 0; i < MOTOR_NUM; i++) {
        digitalWriteFast(MOTOR_PIN[i][0], LOW);
        analogWrite(MOTOR_PIN[i][1], 0);
    }
}
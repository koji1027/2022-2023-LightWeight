#include <Arduino.h>

#define PWM_FREQ 80000
#define Kp 25.0
#define Kd 1.0
#define Ki 10.0

const int MOTOR[4][2] = {{D0, D1}, {D2, D3}, {D4, D5}, {D8, D9}};
const float MOTOR_POS[4] = {3.0 * PI / 4.0, PI / 4.0, 7.0 * PI / 4.0,
                            5.0 * PI / 4.0};

float gyro_angle = 0.0;
float ir_angle = 0.0;
unsigned long long pre_time = 0;
float pre_gyro_angle = 0.0;
float I = 0.0;

void cal(float dir, int speed);

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    analogWriteFreq(80000);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            pinMode(MOTOR[i][j], OUTPUT);
        }
        digitalWrite(MOTOR[i][0], HIGH);
        analogWrite(MOTOR[i][1], 128);
    }
    delay(5000);
}

void loop() {
    cal(PI/2, 50);
    /*
    cal(0, 50);
    delay(2000);
    cal(PI/2, 50);
    delay(2000);
    cal(PI, 50);
    delay(2000);
    cal(PI*3/2, 50);
    delay(2000);
    */
}

void cal(float dir, int speed) {
    float power[4] = {0};
    for (int i = 0; i < 4; i++) {
        power[i] = -sin(dir - MOTOR_POS[i]) * speed;
    }
    int max_power = 0;
    for (int i = 0; i < 4; i++) {
        if (abs(power[i]) > max_power) {
            max_power = abs(power[i]);
        }
    }
    if (max_power != 0) {
        for (int i = 0; i < 4; i++) {
            power[i] = power[i] / max_power * speed;
        }
    }
    float PID, P, D = 0.0;
    unsigned long long now = micros();
    unsigned long long dt = (now - pre_time);
    P = gyro_angle * Kp;
    D = (gyro_angle - pre_gyro_angle) / dt * Kd * 1000000.0;
    if (abs(gyro_angle) < radians(5)) {
        I = 0.0;
    }
    I += gyro_angle * dt * Ki / 1000000.0;
    PID = P + I + D;
    pre_time = now;
    pre_gyro_angle = gyro_angle;
    for (int i = 0; i < 4; i++) {
        power[i] -= PID;
        constrain(power[i], -100, 100);
    }
    for (int i = 0; i < 4; i++) {
        Serial.print(power[i]);
        Serial.print("\t");
        power[i] += 128;
        analogWrite(MOTOR[i][1], power[i]);
    }
    Serial.println();
}

void setup1() { Serial1.begin(115200); }

void loop1() {
    if (Serial1.available() > 4) {
        int recv_data = Serial1.read();
        if (recv_data == 255) {
            int data[4];//[0][1]:gyro, [2][3]:ir
            data[0] = Serial1.read();
            data[1] = Serial1.read();
            int a = data[0] + (data[1] << 8);
            gyro_angle = (a / 100.0) - PI;
            data[2] = Serial1.read();
            data[3] = Serial1.read();
            int b = data[2] + (data[3] << 8);
            ir_angle = (b / 100.0) - PI;
        }
        /*
        Serial.print(gyro_angle);
        Serial.print("\t");
        Serial.println(ir_angle);
        */
    }
}
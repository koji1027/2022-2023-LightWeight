#include "motor.h"

// モーターの初期化
void Motor::begin(void) {
    analogWriteResolution(PWM_RES);  // PWMの分解能を設定
    analogWriteFreq(PWM_FREQ);       // PWMの周波数を設定
    for (uint8_t i = 0; i < MOTOR_NUM; i++) 
    {
        pinMode(PWM_PIN[i], OUTPUT);
        digitalWrite(PWM_PIN[i], HIGH);  // LAP駆動なのでHIGHに固定
        pinMode(DIR_PIN[i], OUTPUT);
        analogWrite(DIR_PIN[i],MAX_PWM / 2);  // LAP駆動なのでDIRでPWM出力 (デューティ比50%)
    }
}

// モーターを制御
void Motor::drive(int *p) {
    uint16_t duty[MOTOR_NUM];  // デューティ比
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        duty[i] = 256 + p[i];  // デューティ比を計算
        digitalWrite(PWM_PIN[i], HIGH);
        analogWrite(DIR_PIN[i], duty[i]);
    }
}

// モーターをブレーキ
void Motor::brake(void) {
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        digitalWrite(PWM_PIN[i], LOW);  // LAP駆動では無い
        analogWrite(DIR_PIN[i], 0);
    }
}

// モーターを解放（現状では，ブレーキと同じ動作をする）
void Motor::release(void) {
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        digitalWrite(PWM_PIN[i], LOW);
        analogWrite(DIR_PIN[i], 0);
    }
}

// 各モーターの出力を計算
void Motor::cal(double move_angle, double gyro_angle, double machine_angle,
                uint8_t speed) {
    double power_ratio[MOTOR_NUM];  // 各モーターの出力比
    double gyro_angle_diff =
        machine_angle -
        gyro_angle;                          // ジャイロの角度と機体の向いているべき方向の差
    unsigned long long now_time = micros();  // 現在の時間
    unsigned long long dt =
        now_time - pre_time;                    // 前回の処理を行った時の時間からの経過時間
    double p, i, d;                             // PID制御のP, I, D
    p = gyro_angle_diff * KP;                   // P制御
    i = gyro_angle_diff * dt * KI / 1000000.0;  // I制御
    d = gyro_angle_diff * 1000000.0 / dt * KD;  // D制御
    double pid = (p + i + d) / PI * 180.0;      // PID制御による出力の合計
    // ジャイロセンサーの角度と機体が向くべき角度との差が45°以下のとき，行きたい方向へ進むように出力を計算する．
    if (abs(gyro_angle_diff) < PI / 4.0) {
        // モーターの出力比を計算
        for (uint8_t i = 0; i < MOTOR_NUM; i++) {
            power_ratio[i] = sin(
                move_angle -
                MOTOR_RAD
                    [i]);  // モーターの出力比を計算（計算法については，別途説明）
        }
        double max = 0;  // 最大の出力比
        for (uint8_t i = 0; i < MOTOR_NUM; i++) {
            if (abs(power_ratio[i]) > max) {
                max = abs(power_ratio[i]);
            }
        }
        // 出力比が最大のモーターの出力がspeedになるように調整
        if (max > 0) {
            for (uint8_t i = 0; i < MOTOR_NUM; i++) {
                power_ratio[i] *= speed / max;
            }
        }
    }
    int power[MOTOR_NUM];  // モーターの出力
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        power_ratio[i] += pid;                                  // PID制御による出力を加算
        power_ratio[i] = constrain(power_ratio[i], -255, 255);  // 出力を制限
        power[i] = (int)power_ratio[i];                         // 出力を整数に変換して保存
    }
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        power[i] = pre_p[i] * POWER_LPF +
                   power[i] * (1 - POWER_LPF);  // 出力にLPFをかける
        pre_p[i] = power[i];                    // 前回の出力を保存
    }
    drive(power);  // モーターを制御
}

// ライン退避
// move_angle: ライン退避する方向
// 機体の角度の補正を行わずに速やかにコート内に退避する
void Motor::esc_line(double move_angle, uint8_t speed) {
    float power_ratio[MOTOR_NUM];  // モーターの出力比
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        power_ratio[i] = sin(
            move_angle -
            MOTOR_RAD
                [i]);  // モーターの出力比を計算（計算法については，別途説明）
    }
    double max = 0;  // 最大の出力比
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        if (abs(power_ratio[i]) > max) {
            max = abs(power_ratio[i]);
        }
    }
    int power[MOTOR_NUM];  // モーターの出力
    // 出力比が最大のモーターの出力がspeedになるように調整
    if (max > 0) {
        for (uint8_t i = 0; i < MOTOR_NUM; i++) {
            power_ratio[i] *= speed / max;
            power[i] = power_ratio[i];
        }
    }
    drive(power);  // モーターを制御
}
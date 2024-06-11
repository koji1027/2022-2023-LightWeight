#include <Arduino.h>

#define MOTOR_NUM 4 // モーターの数
#define PWM_RES 9  // PWMの分解能
#define MAX_PWM pow(2, PWM_RES) // PWMの最大値
#define PWM_FREQ 100000 // PWMの周波数 (100kHz)
#define KP 1.8 // PID制御のPゲイン
#define KI 0.00005 // PID制御のIゲイン
#define KD 0.0 // PID制御のDゲイン
#define POWER_LPF 0 // モーターの出力にかけるLPFの係数

class Motor
{
public:
        void begin(void); // モーターの初期化
        void cal(double move_angle, double gyro_angle, double machine_angle, uint8_t speed); // 各モーターの出力を計算
        void esc_line(double move_angle, uint8_t speed); // ライン退避
        void drive(int *p); // モーターを制御
        void brake(void); // モーターをブレーキ
        void release(void); // モーターを解放

private:
        const int PWM_PIN[MOTOR_NUM] = {D2, D0, D8, D4};  // LAP駆動なのでHIGHに固定
        const int DIR_PIN[MOTOR_NUM] = {D3, D1, D10, D5}; // LAP駆動なのでDIRでPWM出力
        const double MOTOR_RAD[MOTOR_NUM] = {PI / 4.0, PI * 3.0 / 4.0, PI * 5.0 / 4.0, PI * 7.0 / 4.0}; // モーターの角度
        double SIN[628]; // sinのテーブル （計算高速化のため事前に値を保存）
        unsigned long long pre_time = 0; // PID制御のための前回の処理を行った時の時間
        int pre_p[MOTOR_NUM] = {0, 0, 0, 0}; // モーターの出力の前回の値
};

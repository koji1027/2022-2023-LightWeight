// ライブラリのインクルード
#include <Arduino.h>

// 自作ライブラリのインクルード
#include "motor.h"

// 定数の宣言
#define SERIAL_BAUD 115200   // デバッグ用のシリアル通信のボーレート
#define SERIAL1_BAUD 115200  // メインマイコンとの通信のボーレート

// インスタンスの生成
Motor motor;

// グローバル変数の宣言
uint8_t flag = 1;          // モーターの状態に関数フラグ 0: normal, 1: release, 2:brake, 3: ライン退避
double move_angle = 0;     // 進行方向（-PI ~ PI）
double gyro_angle = 0;     // ジャイロの角度（-PI ~ PI）
double machine_angle = 0;  // 機体の向いているべき方向 (-PI ~ PI)
uint8_t speed = 0;         // 速度（0~254）

// 関数の宣言 (setup, loop以外は宣言はここで行い、定義は後で行う)
void uart_recv(void);  // メインマイコンからの通信を受け取る関数

void setup() {
    // put your setup code here, to run once:
    Serial.begin(SERIAL_BAUD);    // デバッグ用のシリアル通信を開始
    motor.begin();                // モーターの初期化
    Serial1.begin(SERIAL1_BAUD);  // メインマイコンとの通信を開始
    while (!Serial1);             // メインマイコンとの通信が確立するまで待機
}

void loop() {
    // put your main code here, to run repeatedly:
    if (Serial1.available()) {  // メインマイコンからの通信があるときデータを受け取る
        uart_recv();
    }
    if (flag == 0) {  // メインマイコンの司令に従ってモーターを制御
        motor.cal(move_angle, gyro_angle, machine_angle, speed);
    } else if (flag == 1) {  // モーターを解放
        motor.release();
    } else if (flag == 2) {  // モーターをブレーキ
        motor.brake();
    } else if (flag == 3) {  // ライン退避
        motor.esc_line(move_angle, speed);
    }
}

// コア1のセットアップ関数(未使用)
void setup1() {
    // put your setup code here, to run once:
}

// コア1のループ関数(未使用)
void loop1() {
    // put your main code here, to run repeatedly:
}

// メインマイコンからの通信を受け取る関数
void uart_recv(void) {
    uint8_t header = Serial1.read();  // ヘッダーを受け取る

    if (header == 255) {                  // ヘッダーが正しいときデータを受け取る
        while (Serial1.available() < 9);  // データが揃うまで待機
        uint8_t buf[8];                   // データを格納するバッファ
        // buf[8] の内部構造について
        // 1bit目 : flag  2bit目 : move_angle（下位7bit）　3bit目 :
        // move_angle（上位3bit） 4bit目 : gyro_angle（下位7bit） 5bit目 :
        // gyro_angle（上位3bit） 6bit目 : machine_angle (下位7bit) 7bit目 :
        // machine_angle (上位3bit) 8bit目 : speed

        for (uint8_t i = 0; i < 8; i++) {  // データを受け取る
            buf[i] = Serial1.read();
        }
        if (Serial1.read() == 254) {  // フッターが正しいときデータを解析
            if (buf[0] != 255 && buf[1] != 255 && buf[2] != 255 && buf[3] != 255 &&
                buf[4] != 255 && buf[5] != 255 && buf[6] != 255 && buf[7] != 255) {  // どのデータもヘッダー（255）と被らないときデータを利用

                flag = buf[0];                                          // モーターの状態に関するフラグを受け取る
                move_angle = (buf[1] + (buf[2] << 7)) / 100.0 - PI;     // 移動方向 0 ~ 200 -> -PI ~ PI
                gyro_angle = (buf[3] + (buf[4] << 7)) / 100.0 - PI;     // ジャイロセンサで得られる機体の向き 0 ~ 200 -> -PI ~ PI
                machine_angle = (buf[5] + (buf[6] << 7)) / 100.0 - PI;  // 機体が向くべき方向 0 ~ 200 -> -PI ~ PI
                speed = buf[7];                                         // 速度 0 ~ 254
            }
        }
    }
}

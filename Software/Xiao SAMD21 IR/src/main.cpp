// ライブラリのインクルード
#include <Arduino.h>

#include "ir.h"  // 赤外線センサー処理用のライブラリ

// 定数の宣言
#define SERIAL_BAUD 115200   // デバッグ用シリアル通信のボーレート
#define SERIAL1_BAUD 115200  // メインマイコンとの通信のボーレート

// 計算量を減らすためにsin,cosの配列を用意
double SIN[628];
double COS[628];

// インスタンスの生成
IR ir;

// 関数の宣言 (setup, loop以外は宣言はここで行い、定義は後で行う)
void uart_send(void);  // 赤外線ボールのデータの送信関数

void setup(void) {
    // put your setup code here, to run once:
    // シリアル通信の初期化
    Serial.begin(SERIAL_BAUD);
    Serial1.begin(SERIAL1_BAUD);
    while (!Serial1);  // メインマイコンとの通信が確立するまで待機

    // 赤外線センサーの初期化
    ir.begin();

    // sin,cosの配列を用意
    for (int i = 0; i < 628; i++) {
        SIN[i] = sin(i / 100.0);
        COS[i] = cos(i / 100.0);
    }
}

void loop(void) {
    // put your main code here, to run repeatedly:
    ir.cal();  // 赤外線センサーの値からボールの角度と距離を計算

    // メインマイコンにデータを送信する
    if (Serial1.available()) {            // メインマイコンからデータが送られてきているか
        uint8_t header = Serial1.read();  // ヘッダーを読み取る
        if (header == 255) {              // ヘッダーが正しいか
            uart_send();                  // データを送信
        }
    }
}

// メインマイコンにデータを送信する関数
void uart_send(void) {
    byte buf[4];            // 送信データを格納するバッファ
    buf[0] = ir.ball_flag;  // ボールの有無のフラグ 0: normal, 1: stop
    // 角度を変換する -PI ~ PI -> 0 ~ 628
    uint16_t tmp = (ir.ir_angle_LPF + PI) * 100.0;
    buf[1] = tmp & 0b0000000001111111;    // 下位7bitを格納
    buf[2] = tmp >> 7;                    // 上位3bitを格納
    tmp = constrain(ir.ir_dist, 0, 254);  // 距離を変換する 0 ~ 254に制限
    buf[3] = (byte)tmp;                   // 距離を格納
    Serial1.write(255);                   // ヘッダーを送信
    Serial1.write(buf, 4);                // データを送信
}

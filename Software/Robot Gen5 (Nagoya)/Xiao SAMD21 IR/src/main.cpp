// ライブラリのインクルード
#include <Arduino.h>

// 自作ライブラリのインクルード
#include "ir.h"

// 定数の宣言
#define SERIAL_BAUD 115200
#define SERIAL1_BAUD 500000
double SIN[628];
double COS[628];

// インスタンスの生成
IR ir;

// グローバル変数の宣言

// 関数の宣言 (setup, loop以外は宣言はここで行い、定義は後で行う)
void uart_send(void); // 赤外線ボールのデータの送信関数

void setup(void)
{
        // put your setup code here, to run once:
        Serial.begin(SERIAL_BAUD);
        Serial1.begin(SERIAL1_BAUD);
        for (int i = 0; i < 628; i++)
        {
                SIN[i] = sin(i / 100.0);
                COS[i] = cos(i / 100.0);
        }
}

void loop(void)
{
        // put your main code here, to run repeatedly:
        if (Serial1.available())
        {
                uint8_t header = Serial1.read();
                if (header == 255)
                {
                        uart_send();
                        Serial.println("send");
                }
        }
}

void uart_send(void)
{
        byte data[4];
        data[0] = ir.ball_flag;                  // 0: normal, 1: stop
        uint16_t tmp = (ir.ir_angle + PI) * 100; //-PI ~ PI -> 0 ~ 628
        data[1] = tmp && 0b0000000011111111;     // 下位7bit
        data[2] = tmp >> 7;                      // 上位2bit
        tmp = constrain(ir.ir_dist, 0, 254);     // 0 ~ 254
        data[3] = (byte)tmp;
        Serial1.write(255); // ヘッダー
        Serial1.write(data, 4);
}

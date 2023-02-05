// ライブラリのインクルード
#include <Arduino.h>

// 自作ライブラリのインクルード

// 定数の宣言
#define SERIAL_BAUD 115200
#define SERIAL1_BAUD 500000
double SIN[628];
double COS[628];

// インスタンスの生成

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
        data[0] = 0;        // 0: normal, 1: stop
        data[1] = 0;        // 0~254
        data[2] = 0;        // -PI ~ PI
        data[3] = 0;        // 0~254
        Serial1.write(255); // ヘッダー
        Serial1.write(data, 4);
}

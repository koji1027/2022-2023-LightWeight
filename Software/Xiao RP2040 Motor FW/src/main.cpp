// ライブラリのインクルード
#include <Arduino.h>

// 自作ライブラリのインクルード
#include "motor.h"

// 定数の宣言
#define SERIAL_BAUD 115200
#define SERIAL1_BAUD 115200

// インスタンスの生成
Motor motor;

// グローバル変数の宣言
// モーターの制御系
uint8_t flag = 1;      // 0: normal, 1: release, 2: brake （0~254）, 3: ライン退避
double move_angle = 0; // 進行方向（-PI ~ PI）
double gyro_angle = 0; // ジャイロの角度（-PI ~ PI）
double machine_angle = 0;
uint8_t speed = 0; // 速度（0~254）

// 関数の宣言 (setup, loop以外は宣言はここで行い、定義は後で行う)
void uart_recv(void); // モーターの制御系の受信関数
void setup()
{
        // put your setup code here, to run once:
        Serial.begin(SERIAL_BAUD);
        motor.begin();
        Serial1.begin(SERIAL1_BAUD);
        while (!Serial1)
                ;
}

void loop()
{
        // put your main code here, to run repeatedly:
        while (0)
        {
                motor.cal(PI / 2.0, 0, 0, 150);
                delay(100);
        }
        if (Serial1.available())
        {
                uart_recv();
        }
        if (flag == 0)
        {
                motor.cal(move_angle, gyro_angle, machine_angle, speed);
        }
        else if (flag == 1)
        {
                motor.release();
        }
        else if (flag == 2)
        {
                motor.brake();
        }
        else if (flag == 3)
        {
                motor.esc_line(move_angle, speed);
        }
        else
        {
        }
}

void setup1()
{
        // put your setup code here, to run once:
}

void loop1()
{
        // put your main code here, to run repeatedly:
}

void uart_recv(void)
{
        uint8_t header = Serial1.read();
        if (header == 255)
        {
                while (Serial1.available() < 9)
                        ;
                uint8_t buf[8];
                // 1bit目 : flag  2bit目 : move_angle（下位7bit）　3bit目 : move_angle（上位3bit）
                // 4bit目 : gyro_angle（下位7bit） 5bit目 : gyro_angle（上位3bit）
                // 6bit目 : machine_angle (下位7bit) 7bit目 : machine_angle (上位3bit) 8bit目 : speed
                for (uint8_t i = 0; i < 8; i++)
                {
                        buf[i] = Serial1.read();
                }
                if (Serial1.read() == 254)
                {
                        if (buf[0] != 255 && buf[1] != 255 && buf[2] != 255 && buf[3] != 255 && buf[4] != 255 && buf[5] != 255 && buf[6] != 255 && buf[7] != 255) // どのデータもヘッダー（255）と被らない
                        {
                                flag = buf[0];
                                move_angle = (buf[1] + (buf[2] << 7)) / 100.0 - PI;    // 0 ~ 200PI -> -PI ~ PI
                                gyro_angle = (buf[3] + (buf[4] << 7)) / 100.0 - PI;    // 0 ~ 200PI -> -PI ~ PI
                                machine_angle = (buf[5] + (buf[6] << 7)) / 100.0 - PI; // 0 ~ 200PI -> -PI ~ PI
                                speed = buf[7];
                                Serial.println("Received");
                        }
                }
        }
}
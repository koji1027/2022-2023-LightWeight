// ライブラリのインクルード
#include <Arduino.h>

// 自作ライブラリのインクルード
#include "led.h"
#include "MPU6050/gyro.h"

// 定数の宣言
#define SERIAL_BAUD 115200
#define MOTOR_BAUD 500000
#define IR_BAUD 500000

// インスタンスの生成
Gyro gyro;
SerialPIO motor(D17, D16, 32); // TX, RX, buffer size
SerialPIO ir(D0, D1, 32);

// グローバル変数の宣言

// モーターの制御系
double move_angle = 0;  // 進行方向（-PI ~ PI）
double gyro_angle = 0;  // ジャイロの角度（-PI ~ PI）
uint8_t speed = 0;      // 速度（0~254）
uint8_t motor_flag = 0; // 0: normal, 1: release, 2 or others: brake（0~254）

// 赤外線センサー制御系
double ir_angle = 0; // 赤外線センサーの角度（-PI ~ PI）
uint8_t ir_dist = 0; // 赤外線センサーの距離（0~254[cm]）
uint8_t ir_flag = 0; // 0: normal, 1: stop (0~254)

// 関数の宣言 (setup, loop以外は宣言はここで行い、定義は後で行う)
void motor_uart_send(void); // モーターの制御系の送信関数
void ir_uart_recv(void);    // 赤外線センサーの制御系の受信関数

void setup(void)
{
        // put your setup code here, to run once:
        Serial.begin(SERIAL_BAUD);
        motor.begin(MOTOR_BAUD);
        while (!motor)
                ;
        ir.begin(IR_BAUD);
        while (!ir)
                ;
        gyro.begin();
        init_led();
}

void loop(void)
{
        // put your main code here, to run repeatedly:
        gyro.getEuler();
        gyro_angle = gyro.angle;
        motor_uart_send();
        ir_uart_recv();
        delay(10);
}

void setup1()
{
        // put your setup code here, to run once:
}

void loop1()
{
        // put your main code here, to run repeatedly:
}

void motor_uart_send(void)
{
        byte buf[6];
        buf[0] = constrain(motor_flag, 0, 254);   // 0: normal, 1: release, 2 or others: brake (0~254)
        uint16_t tmp = (move_angle + PI) * 100.0; //-PI ~ PI -> 0 ~ 200PI
        buf[1] = tmp & 0b0000000001111111;        // 下位7bit
        buf[2] = tmp >> 7;                        // 上位9bit
        tmp = (gyro_angle + PI) * 100.0;          //-PI ~ PI -> 0 ~ 200PI
        buf[3] = tmp & 0b0000000001111111;        // 下位7bit
        buf[4] = tmp >> 7;                        // 上位9bit
        buf[5] = constrain(speed, 0, 254);        // constrain(speed, 0, 254); // 0~254
        motor.write(255);                         // ヘッダー
        motor.write(buf, 6);
}

void ir_uart_recv(void)
{
        ir.write(255); // ヘッダー
        uint8_t recv_fail_count = 0;
        while (ir.available() < 5)
        {
                if (recv_fail_count % 10 == 0 && recv_fail_count != 0 && recv_fail_count < 30)
                {
                        ir.write(255); // ヘッダーの再送信
                }
                else if (recv_fail_count >= 30)
                {
                        Serial.println("ir recv fail");
                        return; // 受信失敗
                }
                delay(1);
                recv_fail_count++;
        }
        byte header = ir.read();
        if (header != 255)
                return;
        byte buf[4];
        ir.readBytes(buf, 4);
        if (buf[0] != 255 && buf[1] != 255 && buf[2] != 255 && buf[3] != 255)
        {
                ir_flag = buf[0];                                // 0: normal, 1: stop (0~254)
                ir_angle = (buf[1] + buf[2] * 128) / 100.0 - PI; // 0 ~ 200PI -> -PI ~ PI
                ir_dist = buf[3];                                // 0~254[cm]
        }
}
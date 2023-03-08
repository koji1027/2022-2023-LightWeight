// ライブラリのインクルード
#include <Arduino.h>
#include <stdlib.h>
#include <Adafruit_SSD1306.h>

// 自作ライブラリのインクルード
#include "led.h"
#include "MPU6050/gyro.h"
#include "line.h"

// 定数の宣言
#define SERIAL_BAUD 115200
#define MOTOR_BAUD 500000
#define IR_BAUD 115200
#define ESP32_BAUD 115200
#define OPENMV_BAUD 115200
#define OpenMV Serial1
#define GOAL_LPF 0.1
#define TWO_THIRDS_PI PI * 2.0 / 3.0
#define CIRC_BASE pow(0.6, 1.0 / 20.0)
#define CIRC_WEIGHT 3.5
#define CIRC_SPEED 100
#define STRAIGHT_SPEED 180
#define GOAL_WEIGHT 2.3

const uint8_t button_pin[3] = {D18, D19, D20}; // ボタンのピン番号

// インスタンスの生成
Gyro gyro;
Line line;
SerialPIO motor(D17, D16, 32); // TX, RX, buffer size
SerialPIO ir(D0, D1, 32);
SerialPIO esp32(D22, D21, 32); // SerialPIOの多用により不具合の可能性あり
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// グローバル変数の宣言

// 全体の制御系
bool game_flag = true;            // ゲームの開始フラグ
bool line_set_threshold_flag = 0; // ラインセンサーの閾値設定フラグ
double battery_voltage = 0;

// モーターの制御系

// ロボットから見た相対的な角度
double move_angle = 0; // 進行方向（-PI ~ PI）
double goal_angle = 0; // ゴールの角度（-PI ~ PI）
double goal_angle_LPF = 0;

// コート全体を俯瞰した時の絶対的な角度
// ゲーム開始時の正面（敵ゴール）方向を０°とし、右回転が正、左回転が負とする。
double abs_move_angle = 0;
double abs_goal_angle = 0;
double abs_line_angle = 0;

// その他
double machine_angle = 0; // ロボットに向かせる角度（-PI ~ PI）
bool goal_flag = 0;       // 0:ゴールなし, 1:ゴールあり
uint8_t speed = 200;      // 速度（0~254）
uint8_t motor_flag = 0;   // 0: normal, 1: release, 2 or others: brake（0~254）

// 赤外線センサー制御系
double ir_angle = 0; // 赤外線センサーの角度（-PI ~ PI）
double abs_ir_angle = 0;
uint8_t ir_dist = 0; // 赤外線センサーの距離（0~254[cm]）
uint8_t ir_flag = 0; // 0: normal, 1: stop (0~254)

// 関数の宣言 (setup, loop以外は宣言はここで行い、定義は後で行う)
void motor_uart_send(void); // モーターの制御系の送信関数
void ir_uart_recv(void);    // 赤外線センサーの制御系の受信関数
void openmv_uart_recv(void);
void push_button(uint gpio, uint32_t events); // ボタンの処理
void line_set_threshold(void);                // 閾値の設定

void setup(void)
{
        // put your setup code here, to run once:
        Serial.begin(SERIAL_BAUD);
        OpenMV.setTX(D12);
        OpenMV.setRX(D13);
        OpenMV.begin(OPENMV_BAUD);
        motor.begin(MOTOR_BAUD);
        while (!motor)
                ;
        ir.begin(IR_BAUD);
        while (!ir)
                ;

        gyro.begin();
        init_led();
        line.begin();
        pinMode(button_pin[0], INPUT_PULLUP);
        pinMode(button_pin[1], INPUT_PULLUP);
        pinMode(button_pin[2], INPUT_PULLUP);
        gpio_set_irq_enabled_with_callback(button_pin[0], GPIO_IRQ_EDGE_FALL, true, push_button);
        gpio_set_irq_enabled_with_callback(button_pin[1], GPIO_IRQ_EDGE_FALL, true, push_button);
        gpio_set_irq_enabled_with_callback(button_pin[2], GPIO_IRQ_EDGE_FALL, true, push_button);
        gpio_set_irq_enabled(button_pin[0], GPIO_IRQ_EDGE_FALL, true);
        gpio_set_irq_enabled(button_pin[1], GPIO_IRQ_EDGE_FALL, true);
        gpio_set_irq_enabled(button_pin[2], GPIO_IRQ_EDGE_FALL, true);
}

void loop(void)
{
        // put your main code here, to run repeatedly:
        if (line_set_threshold_flag)
        {
                line_set_threshold();
        }
        while (game_flag)
        {
                battery_voltage = analogRead(A2) * 3.3 / 1024.0 * 4.0;
                gyro.getEuler();
                ir_uart_recv();
                openmv_uart_recv();
                line.read();
                if (line.on_line)
                {
                        abs_line_angle = line.line_theta + gyro.angle;
                        abs_line_angle = fmod(abs_line_angle, TWO_PI);
                        if (abs_line_angle > PI)
                        {
                                abs_line_angle = abs_line_angle - TWO_PI;
                        }
                }
                abs_ir_angle = ir_angle + gyro.angle;
                abs_ir_angle = fmod(abs_ir_angle, TWO_PI);
                if (abs_ir_angle > PI)
                {
                        abs_ir_angle -= TWO_PI;
                }
                if (goal_flag)
                {
                        machine_angle = goal_angle_LPF * GOAL_WEIGHT;
                }
                else
                {
                        machine_angle = 0;
                }
                if (line.on_line)
                {
                        move_angle = line.line_theta + PI;
                        move_angle = fmod(move_angle, TWO_PI);
                        if (move_angle > PI)
                        {
                                move_angle -= TWO_PI;
                        }
                }
                else
                {
                        if (ir_flag)
                        {
                                float circ_exp = pow(CIRC_BASE, ir_dist);
                                if (abs(abs_ir_angle) < HALF_PI)
                                {
                                        abs_move_angle = abs_ir_angle + constrain(abs_ir_angle * circ_exp * CIRC_WEIGHT, -PI / 2.0, PI / 2.0);
                                        speed = CIRC_SPEED;
                                }
                                else
                                {
                                        abs_move_angle = PI;
                                        speed = STRAIGHT_SPEED;
                                }
                                move_angle = abs_move_angle - machine_angle;
                        }
                        else
                        {
                                move_angle = 0;
                                speed = 0;
                                motor_flag = 2;
                        }
                }
                motor_uart_send();
                delay(10);
        }
}

void setup1()
{
        // put your setup code here, to run once:
}

void loop1()
{
        // put your main code here, to run repeatedly:
        while (game_flag)
        {
        }
}

void motor_uart_send(void)
{
        byte buf[6];
        buf[0] = constrain(motor_flag, 0, 254);   // 0: normal, 1: release, 2 or others: brake (0~254)
        uint16_t tmp = (move_angle + PI) * 100.0; //-PI ~ PI -> 0 ~ 200PI
        buf[1] = tmp & 0b0000000001111111;        // 下位7bit
        buf[2] = tmp >> 7;                        // 上位2bit
        tmp = (gyro.angle + PI) * 100.0;          //-PI ~ PI -> 0 ~ 200PI
        buf[3] = tmp & 0b0000000001111111;        // 下位7bit
        buf[4] = tmp >> 7;                        // 上位3bit
        buf[5] = constrain(speed, 0, 254);        // constrain(speed, 0, 254); // 0~254
        motor.write(255);                         // ヘッダー
        motor.write(buf, 6);
}

void ir_uart_recv(void)
{
        ir.write(255); // ヘッダー
        while (ir.available() < 5)
                ;
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

void push_button(uint gpio, uint32_t events)
{
        if (gpio == button_pin[0])
        {
                gpio_set_irq_enabled(button_pin[0], GPIO_IRQ_EDGE_FALL, false);
                gyro.getEuler();
                gyro.angle_offset = gyro.angle;
                game_flag = 1;
                Serial.println("game start");
                gpio_set_irq_enabled(button_pin[0], GPIO_IRQ_EDGE_FALL, true);
        }
        else if (gpio == button_pin[1])
        {
                gpio_set_irq_enabled(button_pin[1], GPIO_IRQ_EDGE_FALL, false);
                if (!game_flag)
                {
                        line_set_threshold_flag = 1;
                }
                else
                {
                        gpio_set_irq_enabled(button_pin[1], GPIO_IRQ_EDGE_FALL, true);
                }
        }
        else if (gpio == button_pin[2])
        {
                gpio_set_irq_enabled(button_pin[2], GPIO_IRQ_EDGE_FALL, false);
                game_flag = 0;
                Serial.println("game stop");
                gpio_set_irq_enabled(button_pin[2], GPIO_IRQ_EDGE_FALL, true);
        }
}

void line_set_threshold()
{
        Serial.println("Start auto threshold setting");
        Serial.println("Please push middle button to start");
        Serial.println("wait...");
        uint8_t cnt = 0;
        delay(1000);
        while (1)
        {

                if (digitalRead(button_pin[1]) == HIGH)
                {
                        cnt = 0;
                        delay(10);
                }
                else
                {
                        cnt++;
                        delay(10);
                }
                if (cnt > 5)
                {
                        break;
                }
        }
        delay(1000);
        Serial.println("run");
        line.set_threshold();
        delay(1000);
        Serial.println("End auto threshold setting");
        line_set_threshold_flag = false;
        gpio_set_irq_enabled(button_pin[1], GPIO_IRQ_EDGE_FALL, true);
}

void openmv_uart_recv(void)
{
        if (Serial1.available())
        {
                if (Serial1.read() == 255)
                {
                        while (Serial1.available() < 2)
                                ;
                        uint8_t buf[2];
                        buf[0] = Serial1.read();
                        buf[1] = Serial1.read();
                        goal_angle = (buf[0] + buf[1] * 128.0) / 100.0 - PI;
                        goal_angle -= 0.1;
                        if (abs(goal_angle) < TWO_THIRDS_PI)
                        {
                                goal_flag = 1;
                                goal_angle_LPF = goal_angle * GOAL_LPF + goal_angle * (1.0 - GOAL_LPF);
                        }
                        else
                        {
                                goal_flag = 0;
                        }
                        // Serial.println(goal_angle / PI * 180.0);
                }
        }
}

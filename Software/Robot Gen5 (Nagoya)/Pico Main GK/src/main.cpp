// ライブラリのインクルード
#include <Arduino.h>
#include <stdlib.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

// 自作ライブラリのインクルード
#include "MPU6050/gyro.h"
#include "line.h"

// 定数の宣言
#define SERIAL_BAUD 115200
#define MOTOR_BAUD 115200
#define IR_BAUD 115200
#define ESP32_BAUD 115200
#define OPENMV_BAUD 115200
#define OpenMV Serial1
#define GOAL_LPF 0.3
#define PI_THIRDS PI / 3.0
#define TWO_THIRDS_PI PI * 2.0 / 3.0
#define GUARD_SPEED 180
#define BACK_SPEED 140
#define STRAIGHT_SPEED 140
#define ESC_LINE_SPEED 180
#define ATTACK_SPEED 120
#define GOAL_WEIGHT 1.5
#define MAX_SIGNAL 2200
#define MIN_SIGNAL 1000
#define ESC_PIN D14
#define LED_PIN D15
#define LED_NUM 32

const uint8_t button_pin[3] = {D18, D19, D20}; // ボタンのピン番号

// インスタンスの生成
Gyro gyro;
Line line;
SerialPIO motor(D17, D16, 32); // TX, RX, buffer size
SerialPIO ir(D0, D1, 32);
SerialPIO esp32(D22, D21, 32); // SerialPIOの多用により不具合の可能性あり
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Servo esc;

// グローバル変数の宣言

// 全体の制御系
bool game_flag = false;           // ゲームの開始フラグ
bool line_set_threshold_flag = 0; // ラインセンサーの閾値設定フラグ
double battery_voltage = 0;
bool battery_flag = 0;

// モーターの制御系

// ロボットから見た相対的な角度
double move_angle = 0; // 進行方向（-PI ~ PI）
double goal_angle = 0; // ゴールの角度（-PI ~ PI）
double goal_angle_LPF = 0;
double pre_goal_angle[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

// コート全体を俯瞰した時の絶対的な角度
// ゲーム開始時の正面（敵ゴール）方向を０°とし、右回転が正、左回転が負とする。
double abs_move_angle = 0;
double abs_goal_angle = 0;
double abs_goal_angle_LPF = 0;
double abs_line_angle = 0;

// その他
double machine_angle = 0; // ロボットに向かせる角度（-PI ~ PI）
bool goal_flag = 0;       // 0:ゴールなし, 1:ゴールあり
bool attack_flag = 0;
uint8_t goal_flag_ratio = 0;
uint8_t goal_flag_count = 0;

uint8_t speed = 0;      // 速度（0~254）
uint8_t motor_flag = 0; // 0: normal, 1: release, 2 or others: brake（0~254）

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
void bldc_init(void);
void bldc_drive(uint16_t volume);
double normalize_angle(double angle); // 角度を-πからπまでに調整

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
        esp32.begin(ESP32_BAUD);
        gyro.begin();
        line.begin();
        // bldc_init();
        pinMode(button_pin[0], INPUT_PULLUP);
        pinMode(button_pin[1], INPUT_PULLUP);
        pinMode(button_pin[2], INPUT_PULLUP);
        gpio_set_irq_enabled_with_callback(button_pin[0], GPIO_IRQ_EDGE_FALL, true, push_button);
        gpio_set_irq_enabled_with_callback(button_pin[1], GPIO_IRQ_EDGE_FALL, true, push_button);
        gpio_set_irq_enabled_with_callback(button_pin[2], GPIO_IRQ_EDGE_FALL, true, push_button);
        gpio_set_irq_enabled(button_pin[0], GPIO_IRQ_EDGE_FALL, true);
        gpio_set_irq_enabled(button_pin[1], GPIO_IRQ_EDGE_FALL, true);
        gpio_set_irq_enabled(button_pin[2], GPIO_IRQ_EDGE_FALL, true);
        // bldc_drive(0);
        analogReadResolution(10);

        Adafruit_NeoPixel *led = NULL;
        led = new Adafruit_NeoPixel(LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);
        for (int i = 0; i < LED_NUM; i++)
        {
                led->setPixelColor(i, led->Color(255, 0, 0));
        }
        led->setBrightness(200);
        led->show();

        delete (led);
}

void loop(void)
{
        if (line_set_threshold_flag)
        {
                line_set_threshold();
        }
        game_flag = 1;
        while (game_flag)
        {
                motor_flag = 0;
                machine_angle = 0;
                /*
                battery_voltage = analogRead(A2) * 3.3 / 1024.0 * 4.0;;
                Serial.println(battery_voltage);
                if (battery_voltage < 11.0)
                {
                        game_flag = false;
                        battery_flag = true;
                }
                */
                gyro.getEuler();
                ir_uart_recv();
                openmv_uart_recv();
                line.read();
                if (line.on_line)
                {
                        abs_line_angle = line.line_theta + gyro.angle;
                        abs_line_angle = normalize_angle(abs_line_angle);
                }
                abs_ir_angle = ir_angle + gyro.angle;
                abs_ir_angle = normalize_angle(abs_ir_angle);

                // keeper

                if (line.on_line)
                {
                        if (goal_flag)
                        {
                                machine_angle = abs_goal_angle_LPF + PI;
                                machine_angle = normalize_angle(machine_angle);
                                machine_angle *= 1.5;
                                machine_angle = constrain(machine_angle, -PI / 2, PI / 2);
                        }
                        if (!attack_flag)
                        {
                                if (abs(abs_line_angle) < PI_THIRDS)
                                {
                                        line.line_state_flag = 1;
                                        if (abs(ir_angle) > PI / 6)
                                        {
                                                if (abs_ir_angle > 0)
                                                {
                                                        abs_move_angle = HALF_PI;
                                                }
                                                else if (abs_ir_angle < 0)
                                                {
                                                        abs_move_angle = -HALF_PI;
                                                }
                                                move_angle = abs_move_angle - gyro.angle;
                                                // speed = GUARD_SPEED * sin(abs(abs_ir_angle));
                                                speed = 100 + constrain(80 * ir_dist * abs(sin(abs_ir_angle)) / 50.0, 0, 80);
                                        }
                                        else
                                        {
                                                // attack_flag = 1;
                                                move_angle = 0;
                                                speed = STRAIGHT_SPEED;
                                        }
                                }
                                else if (abs_line_angle > 0 && abs_line_angle < PI * 165 / 180)
                                {
                                        // move_angle = HALF_PI;
                                        abs_move_angle = PI_THIRDS;
                                        move_angle = abs_move_angle - gyro.angle;
                                }
                                else if (abs_line_angle < 0 && abs_line_angle > -PI * 165 / 180)
                                {
                                        // move_angle = -HALF_PI;
                                        abs_move_angle = -PI_THIRDS;
                                        move_angle = abs_move_angle - gyro.angle;
                                }
                                else
                                {
                                        line.line_state_flag = 3;
                                        abs_move_angle = PI;
                                        move_angle = abs_move_angle - gyro.angle;
                                        speed = BACK_SPEED;
                                }
                        }
                        else
                        {
                                if (abs(ir_angle) > PI / 6.0)
                                {
                                        attack_flag = 0;
                                        move_angle = PI;
                                        speed = BACK_SPEED;
                                }
                                else
                                {
                                        move_angle = 0;
                                        speed = ATTACK_SPEED;
                                }
                                /*if (abs_line_angle > 0 && abs_line_angle < PI * 165 / 180)
                                {
                                        //move_angle = HALF_PI;
                                        move_angle = PI_THIRDS;
                                        attack_flag = 0;
                                }
                                else if (abs_line_angle < 0 && abs_line_angle > -PI * 165 / 180)
                                {
                                        //move_angle = -HALF_PI;
                                        move_angle = -PI_THIRDS;
                                        attack_flag = 0;
                                }*/
                        }
                }
                else
                {
                        if (attack_flag)
                        {
                                if (goal_flag)
                                {
                                        move_angle = 0;
                                        speed = ATTACK_SPEED;
                                        if (abs(ir_angle) > PI / 6.0)
                                        {
                                                attack_flag = 0;
                                                if (abs(line.line_theta) <= HALF_PI && line.on_line)
                                                {
                                                        line.line_state_flag = 1;
                                                }
                                                else
                                                {
                                                        line.line_state_flag = 3;
                                                }
                                        }
                                }
                                else
                                {
                                        attack_flag = 0;
                                        line.line_state_flag = 3;
                                }
                        }
                        if (line.line_state_flag == 1)
                        {
                                move_angle = 0;
                        }
                        else if (line.line_state_flag == 3)
                        {
                                move_angle = PI;
                        }
                }

                move_angle = normalize_angle(move_angle);

                motor_uart_send();
                if (Serial.available())
                {
                        uint16_t volume = Serial.readStringUntil('\n').toInt();
                        bldc_drive(volume);
                }
                delay(10);
        }
        move_angle = 0;
        speed = 0;
        motor_flag = 2;
        motor_uart_send();
}

/*
void aaa(void)
{
        // put your main code here, to run repeatedly:
        move_angle = 0.0;
        gyro.getEuler();
        speed = 100;
        motor_uart_send();
        Serial.println(move_angle);
        esp32.println(move_angle);
        delay(10);
}
*/

void motor_uart_send(void)
{
        byte buf[8];
        buf[0] = constrain(motor_flag, 0, 254);   // 0: normal, 1: release, 2 or others: brake (0~254)
        uint16_t tmp = (move_angle + PI) * 100.0; //-PI ~ PI -> 0 ~ 200PI
        buf[1] = tmp & 0b0000000001111111;        // 下位7bit
        buf[2] = tmp >> 7;                        // 上位2bit
        tmp = (gyro.angle + PI) * 100.0;          //-PI ~ PI -> 0 ~ 200PI
        buf[3] = tmp & 0b0000000001111111;        // 下位7bit
        buf[4] = tmp >> 7;                        // 上位3bit
        tmp = (machine_angle + PI) * 100.0;
        buf[5] = tmp & 0b0000000001111111; // 下位7bit
        buf[6] = tmp >> 7;                 // 上位3bit
        buf[7] = constrain(speed, 0, 254); // constrain(speed, 0, 254); // 0~254
        motor.write(255);                  // ヘッダー
        motor.write(buf, 8);
        motor.write(254);
        // Serial.println("Send");
}

void ir_uart_recv(void)
{
        ir.write(255); // ヘッダー
        unsigned long long request_time = micros();
        while (!ir.available())
        {
                if (micros() - request_time > 10000)
                {
                        break;
                }
        }
        byte header = ir.read();
        if (header != 255)
                return;
        unsigned long long wait_time = micros();
        while (ir.available() < 4)
        {
                if (micros() - wait_time > 10000)
                {
                        while (ir.available())
                        {
                                ir.read();
                        }
                        break;
                }
        }
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
                // Serial.println("game start");
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
                // Serial.println("game stop");
                speed = 0;
                move_angle = 0;
                motor_flag = 1;
                motor_uart_send();
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
        // Serial.println("run");
        line.set_threshold();
        delay(1000);
        // Serial.println("End auto threshold setting");
        line_set_threshold_flag = false;
        gpio_set_irq_enabled(button_pin[1], GPIO_IRQ_EDGE_FALL, true);
}

void openmv_uart_recv(void)
{
        if (Serial1.available())
        {
                if (Serial1.read() == 255)
                {
                        while (Serial1.available() < 3)
                                ;
                        uint8_t buf[3];
                        buf[0] = Serial1.read();
                        buf[1] = Serial1.read();
                        buf[2] = Serial1.read();
                        goal_flag = buf[2];
                        if (goal_flag)
                        {
                                double pre_goal_angle = goal_angle;
                                goal_angle = (buf[0] + buf[1] * 128.0) / 100.0 - PI;
                                if (abs(goal_angle) > PI)
                                {
                                        goal_flag = 0;
                                }
                                if (abs(goal_angle - pre_goal_angle) > PI / 4.0)
                                {
                                        goal_flag = 0;
                                }
                                Serial.println(goal_angle / PI * 180.0);
                                abs_goal_angle = goal_angle + gyro.angle;
                                abs_goal_angle_LPF = abs_goal_angle_LPF * GOAL_LPF + abs_goal_angle * (1.0 - GOAL_LPF);
                                abs_goal_angle_LPF = normalize_angle(abs_goal_angle_LPF);
                        }
                }
        }
}

void bldc_init(void)
{
        esc.attach(ESC_PIN);
        esc.writeMicroseconds(MAX_SIGNAL);
        delay(2000);
        esc.writeMicroseconds(MIN_SIGNAL);
        delay(2000);
}

void bldc_drive(uint16_t volume)
{
        esc.writeMicroseconds(volume);
}

double normalize_angle(double angle)
{
        if (angle > 0)
        {
                angle = fmod(angle, TWO_PI);
                if (angle > PI)
                {
                        angle = angle - TWO_PI;
                }
        }
        else if (angle < 0)
        {
                angle = fmod(abs(angle), TWO_PI);
                if (angle > PI)
                {
                        angle = angle - TWO_PI;
                }
                angle = -angle;
        }
        return angle;
};
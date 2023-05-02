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
#define GUARD_SPEED 180
#define BACK_SPEED 180
#define STRAIGHT_SPEED 180
#define ESC_LINE_SPEED 200
#define ATTACK_SPEED 180
#define GOAL_WEIGHT 1.5
#define MAX_SIGNAL 2200
#define MIN_SIGNAL 1000
#define LED_PIN D15
#define LED_NUM 32
#define SLIDE_SPEED 180
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

#define PI_THIRDS PI / 3.0
#define TWO_THIRDS_PI PI * 2.0 / 3.0
#define PI_NINTH PI / 9.0
#define PI_FOURTH PI / 4.0
#define PI_THREE_FOURTH PI * 3.0 / 4.0
#define PI_SIXTH PI / 6.0
#define PI_165_180 PI * 165 / 180
#define EIGHT_NINTH_PI PI * 8.0 / 9.0

const uint8_t button_pin[3] = {D18, D19, D20}; // ボタンのピン番号

// インスタンスの生成
Gyro gyro;
Line line;
SerialPIO motor(D17, D16, 32); // TX, RX, buffer size
SerialPIO ir(D0, D1, 32);

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
double abs_goal_angle_inv = 0;
double abs_line_angle = 0;

// その他
double machine_angle = 0; // ロボットに向かせる角度（-PI ~ PI）
bool goal_flag = 0;       // 0:ゴールなし, 1:ゴールあり
bool attack_flag = 0;
bool corner_flag = 0;
uint8_t goal_flag_ratio = 0;
uint8_t goal_flag_count = 0;
uint8_t goal_dist = 0;

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
void line_set_threshold(void); // 閾値の設定
void bldc_init(void);
void bldc_drive(uint16_t volume);
double normalize_angle(double angle); // 角度を-πからπまでに調整
void line_set_threshold();

void setup(void)
{
        // put your setup code here, to run once:
        Serial.begin(SERIAL_BAUD);
        gyro.begin();
        motor.begin(MOTOR_BAUD);
        while (!motor)
                ;
        ir.begin(IR_BAUD);
        while (!ir)
                ;
        OpenMV.setTX(D12);
        OpenMV.setRX(D13);
        OpenMV.begin(OPENMV_BAUD);
        // while (!OpenMV.available())
        //         ;
        OpenMV.read();
        line.begin();
        pinMode(button_pin[0], INPUT_PULLUP);
        pinMode(button_pin[1], INPUT_PULLUP);
        pinMode(button_pin[2], INPUT_PULLUP);
        Adafruit_NeoPixel *led = NULL;
        led = new Adafruit_NeoPixel(LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);
        for (int i = 0; i < LED_NUM; i++)
        {
                led->setPixelColor(i, led->Color(255, 0, 0));
        }
        led->setBrightness(255);
        led->show();

        delete (led);
}

void loop(void)
{
        while (0)
        {
                /*
                line.read();
                for (int i = 16; i < 32; i++)
                {
                        Serial.print(line.sensor_value[i]);
                        Serial.print("\t");
                }
                Serial.println();
                */
                ir_uart_recv();
                Serial.println(ir_angle);
                delay(50);
        }
        while (game_flag)
        {
                unsigned long long time = micros();
                gyro.getEuler();
                ir_uart_recv();
                openmv_uart_recv();
                line.read();
                motor_flag = 0;
                machine_angle = 0;
                speed = 150;
                if (line.on_line)
                {
                        abs_line_angle = line.line_theta + gyro.angle;
                        abs_line_angle = normalize_angle(abs_line_angle);
                }
                abs_ir_angle = ir_angle + gyro.angle;
                abs_ir_angle = normalize_angle(abs_ir_angle);
                if (line.on_line)
                {
                        if (!attack_flag)
                        {
                                if (abs(abs_line_angle) < PI_THIRDS)
                                {
                                        line.line_state_flag = 1;
                                        if (abs_ir_angle > PI_SIXTH && abs_ir_angle < EIGHT_NINTH_PI)
                                        {
                                                abs_move_angle = HALF_PI;
                                                speed = SLIDE_SPEED - 15 * cos(abs_ir_angle);
                                        }
                                        else if (abs_ir_angle < -PI_SIXTH && abs_ir_angle > -EIGHT_NINTH_PI)
                                        {
                                                abs_move_angle = -HALF_PI;
                                                speed = SLIDE_SPEED - 15 * cos(abs_ir_angle);
                                        }
                                        else if (abs(abs_ir_angle) >= EIGHT_NINTH_PI)
                                        {
                                                speed = 0;
                                        }
                                        else
                                        {
                                                abs_move_angle = 0;
                                                speed = ATTACK_SPEED;
                                        }
                                        move_angle = abs_move_angle - gyro.angle;
                                }
                                else if (abs_line_angle > 0 && abs_line_angle < PI_165_180)
                                {
                                        abs_move_angle = PI_FOURTH;
                                        move_angle = abs_move_angle - gyro.angle;
                                        speed = ESC_LINE_SPEED;
                                }
                                else if (abs_line_angle < 0 && abs_line_angle > -PI_165_180)
                                {
                                        abs_move_angle = -PI_FOURTH;
                                        move_angle = abs_move_angle - gyro.angle;
                                        speed = ESC_LINE_SPEED;
                                }
                                else
                                {
                                        line.line_state_flag = 3;
                                        if (abs_ir_angle > 0)
                                        {
                                                abs_move_angle = PI_THREE_FOURTH;
                                                speed = SLIDE_SPEED - 15 * cos(abs_ir_angle);
                                        }
                                        else if (abs_ir_angle < 0)
                                        {
                                                abs_move_angle = -PI_THREE_FOURTH;
                                                speed = SLIDE_SPEED - 15 * cos(abs_ir_angle);
                                        }
                                        else
                                        {
                                                abs_move_angle = PI;
                                                speed = ATTACK_SPEED;
                                        }
                                        move_angle = abs_move_angle - gyro.angle;
                                }
                        }
                        else
                        {
                                if (abs(ir_angle) > PI_SIXTH)
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
                                        if (abs(ir_angle) > PI_SIXTH)
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
                        /*
                        if (!goal_flag || goal_dist > 100)
                        {
                                // Serial.println("Yo");
                                if (line.line_state_flag != 1)
                                {
                                        move_angle = PI;
                                        line.line_state_flag = 3;
                                        speed = BACK_SPEED;
                                }
                        }*/
                        /*
                        // ここ以下、新規追加。動作未確認
                        else if (goal_flag && abs(abs_goal_angle_inv) < PI / 12.0)
                        {
                                move_angle = PI;
                                line.line_state_flag = 3;
                                speed = BACK_SPEED;
                                Serial.print("B");
                        }
                        else if (goal_flag && abs_goal_angle_inv >= PI / 12.0 && abs_goal_angle_inv < HALF_PI)
                        {
                                move_angle = PI_FOURTH;
                                line.line_state_flag = 3;
                                speed = BACK_SPEED;
                                Serial.print("C");
                        }
                        else if (goal_flag && abs_goal_angle_inv <= -PI / 12.0 && abs_goal_angle_inv > -HALF_PI)
                        {
                                move_angle = -PI_FOURTH;
                                line.line_state_flag = 3;
                                speed = BACK_SPEED;
                                Serial.print("D");
                        }

                        // ここまで.
                        */
                        else if (line.line_state_flag == 1)
                        {
                                line.line_state_flag = 1;
                                if (abs_ir_angle > PI_NINTH)
                                {
                                        abs_move_angle = PI_FOURTH;
                                        speed = SLIDE_SPEED - 15 * cos(abs_ir_angle);
                                }
                                else if (abs_ir_angle < -PI_NINTH)
                                {
                                        abs_move_angle = -PI_FOURTH;
                                        speed = SLIDE_SPEED - 15 * cos(abs_ir_angle);
                                }
                                else
                                {
                                        abs_move_angle = 0;
                                        speed = ATTACK_SPEED;
                                }
                                move_angle = abs_move_angle - gyro.angle;
                        }
                        else if (line.line_state_flag == 3)
                        {
                                line.line_state_flag = 3;
                                if (abs_ir_angle > PI_NINTH)
                                {
                                        abs_move_angle = PI_THREE_FOURTH;
                                        speed = SLIDE_SPEED - 15 * cos(abs_ir_angle);
                                }
                                else if (abs_ir_angle < -PI_NINTH)
                                {
                                        abs_move_angle = -PI * 2.0 / 3.0;
                                        speed = SLIDE_SPEED - 15 * cos(abs_ir_angle);
                                }
                                else
                                {
                                        abs_move_angle = PI;
                                        speed = ATTACK_SPEED;
                                }
                                move_angle = abs_move_angle - gyro.angle;
                        }
                }
                if (ir_flag)
                {
                        move_angle = normalize_angle(move_angle);
                }
                else
                {
                        speed = 0;
                }
                motor_uart_send();
                if (digitalRead(button_pin[0]) == LOW)
                {
                        int cnt = 0;
                        while (1)
                        {
                                if (digitalRead(button_pin[0]) == LOW)
                                {
                                        cnt++;
                                        if (cnt == 100)
                                        {
                                                break;
                                        }
                                }
                                else
                                {
                                        cnt = 0;
                                        break;
                                }
                                delay(2);
                        }
                        if (cnt == 100)
                        {
                                game_flag = 0;
                                Serial.println("Game Stop");
                        }
                }
                // double fps = 1000000.0 / (micros() - time);
                //  Serial.println(fps);
        }
        if (digitalRead(button_pin[1]) == LOW)
        {
                int cnt = 0;
                while (1)
                {
                        if (digitalRead(button_pin[1]) == LOW)
                        {
                                cnt++;
                                if (cnt == 50)
                                {
                                        break;
                                }
                        }
                        else
                        {
                                cnt = 0;
                                break;
                        }
                        delay(2);
                }
                if (cnt == 50)
                {
                        game_flag = 1;
                        Serial.println("Game Start");
                }
        }
        move_angle = 0;
        speed = 0;
        motor_flag = 2;
        motor_uart_send();
}

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
                // speed = 0;
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
        delay(1000);
        Serial.println("run");
        line.set_threshold();
        delay(1000);
        Serial.println("End auto threshold setting");
}

void openmv_uart_recv(void)
{
        if (Serial1.available())
        {
                if (Serial1.read() == 255)
                {
                        while (Serial1.available() < 4)
                                ;
                        uint8_t buf[3];
                        buf[0] = Serial1.read();
                        buf[1] = Serial1.read();
                        buf[2] = Serial1.read();
                        buf[3] = Serial1.read();
                        goal_flag = buf[2];

                        if (goal_flag)
                        {
                                double pre_goal_angle = abs_goal_angle;
                                goal_angle = (buf[0] + buf[1] * 128.0) / 100.0 - PI;
                                goal_dist = buf[3];
                                if (abs(goal_angle) > PI)
                                {
                                        goal_flag = 0;
                                }
                                // Serial.println(goal_angle / PI * 180.0);
                                abs_goal_angle = goal_angle + gyro.angle;
                                abs_goal_angle = normalize_angle(abs_goal_angle);
                                if (abs(abs_goal_angle - pre_goal_angle) > PI / 4.0 && goal_flag)
                                {
                                        goal_flag = 0;
                                }
                                goal_dist = buf[3];
                                abs_goal_angle_inv = normalize_angle(abs_goal_angle + PI);
                        }
                        // Serial.println(abs_goal_angle_inv);
                        //  Serial.println(goal_dist);
                }
        }
}

double normalize_angle(double angle)
{
        while (angle > PI)
        {
                angle -= TWO_PI;
        }
        while (angle <= -PI)
        {
                angle += TWO_PI;
        }
        return angle;
};
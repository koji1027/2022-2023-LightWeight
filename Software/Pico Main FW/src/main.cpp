// Arduinoライブラリのインクルード
#include <Arduino.h>

// ライブラリのインクルード
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <stdlib.h>

// 自作ライブラリのインクルード
#include "MPU6050/gyro.h"
#include "line.h"

// 定数の宣言
#define SERIAL_BAUD 115200  // デバッグ用のシリアル通信のボーレート
#define MOTOR_BAUD 115200   // モーターの制御マイコンとの通信のボーレート
#define IR_BAUD 115200      // 赤外線センサー制御マイコンとの通信のボーレート
#define ESP32_BAUD 115200   // ESP32(Bluetooth通信)との通信のボーレート
#define OPENMV_BAUD 115200  // OpenMVとの通信のボーレート
#define OpenMV Serial1      // OpenMVとの通信のシリアルポート
// よく使う角度を定数に保存しておく
#define PI_THIRDS PI / 3.0
#define TWO_THIRDS_PI PI * 2.0 / 3.0
#define PI_EIGHTHS PI / 8.0
#define THREE_EIGHTHS_PI PI * 3.0 / 8.0
#define FIVE_EIGHTHS_PI PI * 5.0 / 8.0
// 回り込みのための計算式の係数
#define CIRC_BASE pow(0.6, 1.0 / 20.0)
#define CIRC_WEIGHT 3.5
// 機体の速度
#define CIRC_SPEED 160
#define STRAIGHT_SPEED 150
#define ESC_LINE_SPEED 190
// ゴールの角度について
#define GOAL_WEIGHT 1.4　  //OpenMVからのゴールの角度を使用する際の重み
#define MAX_SIGNAL 2200    // OpenMVからの信号の最大値
#define MIN_SIGNAL 1000    // OpenMVからの信号の最小値
// ピン番号の定義
#define ESC_PIN D14  // ESC(ブラシレスモータードライバー)の制御ピン番号
#define LED_PIN D15  // LEDのピン番号
#define LED_NUM 32   // ラインセンサのLEDの数
// 操作モニターの設定
#define SCREEN_WIDTH 128     // 画面の幅
#define SCREEN_HEIGHT 64     // 画面の高さ
#define SCREEN_ADDRESS 0x3C  // SD1306のアドレス

const uint8_t button_pin[3] = {D18, D19, D20};  // 操作ボタンのピン番号

// インスタンスの生成
Gyro gyro;
Line line;
// Raspberry Pi pico では，PIOという内臓のコンピュータを利用することで，ハードウェアシリアルに似たソフトウェアシリアルを実装されている．
// ハードウェアシリアルが足りのないので，ソフトウェアシリアルを利用する．
SerialPIO motor(D17, D16, 32);  // SerialPIO name(TX, RX, buffer size) の形で宣言する．
SerialPIO ir(D0, D1, 32);
// SerialPIO esp32(D22, D21, 32); //これを使うとプログラムがバグる（詳細は忘れた）．多分，SerialPIOを使いすぎている．
Servo esc;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);  // モニター

// 全体の制御に関する変数
bool game_flag = false;            // ゲーム中かどうかのフラグ　0:ゲーム中でない, 1:ゲーム中
bool line_set_threshold_flag = 0;  // ラインセンサーの閾値設定フラグ（何に使っていたか忘れた）
double battery_voltage = 0;        // バッテリーの電圧(使ってない)
bool battery_flag = 0;             // バッテリーの電圧が低電圧かどうかのフラグ(使ってない)

// モーターの制御系
// ロボットから見た相対的な角度
double move_angle = 0;     // 進行方向（-PI ~ PI）
double goal_angle = 0;     // ゴールの角度（-PI ~ PI）
uint16_t goal_size = 0;    // ゴールのサイズ(なんで実装したか忘れた)
double machine_angle = 0;  // ロボットに向かせる角度（-PI ~ PI）

// コート全体を俯瞰した時の絶対的な角度
// ゲーム開始時の正面（敵ゴール）方向を０°とし、右回転が正、左回転が負とする。
double abs_move_angle = 0;  // 進行方向（-PI ~ PI）（絶対角度）
double abs_goal_angle = 0;  // ゴールの角度（-PI ~ PI）（絶対角度）
double abs_line_angle = 0;  // ラインの角度（-PI ~ PI）（絶対角度）

bool goal_flag = 0;           // 0:ゴールなし, 1:ゴールあり
uint8_t goal_flag_ratio = 0;  // 忘れた(斎藤に聞けばわかるかも)
uint8_t goal_flag_count = 0;  // 忘れた(斎藤に聞けばわかるかも)
uint8_t speed = 0;            // 速度（0~254）
uint8_t motor_flag = 0;       // モーターの状態についてのフラグ 0: normal, 1: release, 2 or others: brake（0~254）

unsigned long long display_refresh_time = 0;  // ディスプレイのリフレッシュ時間

// fpsを算出するための変数
unsigned long long start_time = 0;
unsigned long long end_time = 0;
int fps = 0;

// 赤外線センサー制御系
double ir_angle = 0;      // 赤外線センサーの角度（-PI ~ PI）
double abs_ir_angle = 0;  // 赤外線センサーの角度（-PI ~ PI）（絶対角度）
uint8_t ir_dist = 0;      // 赤外線センサーの距離（0~254[cm]）(使い物にならなかった)
uint8_t ir_flag = 0;      // ボールが検知されたかどうかのフラグ 0: 有り, 1: なし (0~254)

// 関数の宣言 (setup, loop以外は宣言はここで行い、定義は後で行う)
void motor_uart_send(void);                    // モーター制御系の送信関数
void ir_uart_recv(void);                       // 赤外線センサー制御系の受信関数
void openmv_uart_recv(void);                   // OpenMVとの通信の受信関数
void push_button(uint gpio, uint32_t events);  // ボタンの処理
void line_set_threshold(void);                 // 閾値の自動設定
void bldc_init(void);                          // ブラシレスモータードライバーの初期化
void bldc_drive(uint16_t volume);              // ブラシレスモータードライバーの制御
double normalize_angle(double angle);          // 角度を-πからπまでに調整
void esc_line(void);                           // ライン退避
void refresh_display(void);                    // ディスプレイのリフレッシュ

void setup(void) {
    // put your setup code here, to run once:
    // 各シリアル通信の開始
    Serial.begin(SERIAL_BAUD);
    OpenMV.setTX(D12);  // OpenMVとの通信のピン番号の設定
    OpenMV.setRX(D13);  // OpenMVとの通信のピン番号の設定
    OpenMV.begin(OPENMV_BAUD);
    motor.begin(MOTOR_BAUD);
    while (!motor);  // モーター制御マイコンとの通信の確立
    ir.begin(IR_BAUD);
    while (!ir);  // 赤外線センサー制御マイコンとの通信の確立
    // 6軸センサとラインセンサの初期化
    gyro.begin();
    line.begin();
    // bldc_init();　// ブラシレスモータードライバーの初期化(ドリブラー非搭載となったので不要)
    // bldc_drive(0);

    // ボタンの初期化
    pinMode(button_pin[0], INPUT_PULLUP);
    pinMode(button_pin[1], INPUT_PULLUP);
    pinMode(button_pin[2], INPUT_PULLUP);

    analogReadResolution(10);  // アナログリードの解像度を10bitに設定

    // NeoPixelの初期化(ラインセンサのLED)
    // 使い方はググってくれ
    Adafruit_NeoPixel *led = NULL;
    led = new Adafruit_NeoPixel(LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);
    for (int i = 0; i < LED_NUM; i++) {
        led->setPixelColor(i, led->Color(255, 0, 0));
    }
    led->setBrightness(200);  // LEDの明るさを設定(0~255)
    led->show();

    delete (led);  // メモリの解放(多分いらない．昔Neopixelが原因でコードが停止するバグがあったときに一応入れた．)
}

// RP2040(マイコン)のコア0のsetupとloop
void loop(void) {
    if (line_set_threshold_flag) {  // 必要があればラインセンサーの閾値設定を行う
        line_set_threshold();
    }
    while (game_flag) {                                         // このwhile文の中でゲームが行われる
        start_time = micros();                                  // fpsの計算のための時間計測
        motor_flag = 0;                                         // モーターの状態を初期化
        battery_voltage = analogRead(A2) * 3.3 / 1023.0 * 4.0;  // バッテリーの電圧を計測(役に立たない．使ってない)
        gyro.getEuler();                                        // 6軸センサの角度(オイラー角）を取得．（オイラー角は角度の表現方法の一つ．自分で調べな．）
        if (millis() - display_refresh_time > 100) {            // ディスプレイのリフレッシュ（100msごとに行う）．while文の中で毎回行うとディスプレイとの通信に時間がかかってボトルネックになってしまう．
            refresh_display();
        }
        ir_uart_recv();      // 赤外線センサー制御系の受信
        openmv_uart_recv();  // OpenMVとの通信の受信
        line.read();         // ラインセンサーの読み取り
        // line.debug(); // ラインセンサーのデバッグ

        if (line.on_line) {                                 // ライン上にいる場合
            abs_line_angle = line.line_theta + gyro.angle;  // ラインの角度(絶対角度)を取得
            // ラインの角度が-π ~ πになるように調整
            if (abs_line_angle > PI) {
                abs_line_angle -= TWO_PI;
            } else if (abs_line_angle <= -PI) {
                abs_line_angle += TWO_PI;
            }
        }
        // 赤外線センサーの角度（絶対角度）を取得
        abs_ir_angle = ir_angle + gyro.angle;
        // 赤外線センサーの角度が-π ~ πになるように調整
        abs_ir_angle = fmod(abs_ir_angle, TWO_PI);
        if (abs_ir_angle > PI) {
            abs_ir_angle -= TWO_PI;
        }
        if (abs_ir_angle < -PI) {
            abs_ir_angle += TWO_PI;
        }
        if (line.on_line) {  // ライン上にいる場合
            // ラインの角度による分類を以下に示す
            // 前： -PI/8 ~ PI/8, 右前: PI/8 ~ 3PI/8, 右: 3PI/8 ~ 5PI/8, 右後ろ: 5PI/8 ~ 7PI/8,
            // 後ろ: 7PI/8 ~ -7PI/8, 左後ろ: -7PI/8 ~ -5PI/8, 左: -5PI/8 ~ -3PI/8, 左前: -3PI/8 ~ -PI/8
            // ライン退避がめっちゃぐちゃぐちゃになっていしまったのは申し訳ない．大会当日はコードを綺麗に整える余裕がなかった．（やってみればわかる．）
            if (abs_line_angle > PI * 3.0 / 8.0 && abs_line_angle <= PI * 5.0 / 8.0)  // ラインが右にある
            {
                if (line.line_state_flag == 0 || line.line_state_flag == 2) {
                    line.line_state_flag = 2;
                    if (abs_ir_angle <= PI / 2.0 && abs_ir_angle >= 0) {
                        abs_move_angle = -PI / 9.0;
                        move_angle = abs_move_angle - gyro.angle;
                        speed = 100;
                        motor_flag = 0;
                        // Serial.println("front");
                    } else if (abs_ir_angle >= PI / 2.0 || abs_ir_angle <= -PI / 3.0) {
                        abs_move_angle = -PI * 8.0 / 9.0;
                        move_angle = abs_move_angle - gyro.angle;
                        speed = 100;
                        motor_flag = 0;
                        // Serial.println("back");
                    } else if (abs_ir_angle < 0 && abs_ir_angle >= -PI / 3.0) {
                        line.on_line = false;
                        // Serial.println("front");
                    }
                } else {
                    esc_line();
                }
            } else if (abs_line_angle <= -PI * 3.0 / 8.0 && abs_line_angle > -PI * 5.0 / 8.0)  // ラインが左にある
            {
                // Serial.println("left");
                if (line.line_state_flag == 0 || line.line_state_flag == 4) {
                    line.line_state_flag = 4;
                    if (abs_ir_angle >= -PI / 2.0 && abs_ir_angle <= 0) {
                        abs_move_angle = PI / 9.0;
                        move_angle = abs_move_angle - gyro.angle;
                        speed = 100;
                        motor_flag = 0;
                        // Serial.println("front");
                    } else if (abs_ir_angle <= -PI / 2.0 || abs_ir_angle >= PI / 3.0) {
                        abs_move_angle = PI * 8.0 / 9.0;
                        move_angle = abs_move_angle - gyro.angle;
                        speed = 100;
                        motor_flag = 0;
                        // Serial.println("back");
                    } else if (abs_ir_angle > 0 && abs_ir_angle <= PI / 3.0) {
                        line.on_line = false;
                        // Serial.println("front");
                    }
                } else {
                    esc_line();
                }
            } else if (abs_line_angle <= PI / 8.0 && abs_line_angle > -PI / 8.0)  // ラインが前にある
            {
                // Serial.println("front");
                if (line.line_state_flag == 0 || line.line_state_flag == 1) {
                    line.line_state_flag = 1;
                    esc_line();
                } else {
                    esc_line();
                }
            } else if (abs_line_angle <= PI * 3.0 / 8.0 && abs_line_angle > PI / 8.0)  // ラインが右前にある
            {
                // Serial.println("right front");
                if (line.line_state_flag == 0 || line.line_state_flag == 5) {
                    line.line_state_flag = 5;
                    esc_line();
                } else {
                    esc_line();
                }
            } else if (abs_line_angle <= -PI / 8.0 && abs_line_angle > -PI * 3.0 / 8.0)  // ラインが左前にある
            {
                // Serial.println("left front");
                if (line.line_state_flag == 0 || line.line_state_flag == 7) {
                    line.line_state_flag = 7;
                    esc_line();
                } else {
                    esc_line();
                }
            } else if (abs_line_angle <= PI * 7.0 / 8.0 && abs_line_angle > PI * 5.0 / 8.0)  // ラインが右後ろにある
            {
                // Serial.println("right back");
                if (line.line_state_flag == 0 || line.line_state_flag == 6) {
                    line.line_state_flag = 6;
                    esc_line();
                } else {
                    esc_line();
                }
            } else if (abs_line_angle <= -PI * 5.0 / 8.0 && abs_line_angle > -PI * 7.0 / 8.0)  // ラインが左後にある
            {
                // Serial.println("left back");
                if (line.line_state_flag == 0 || line.line_state_flag == 8) {
                    line.line_state_flag = 8;
                    esc_line();
                } else {
                    esc_line();
                }
            } else  // ラインが後ろにある
            {
                // Serial.println("back");
                if (line.line_state_flag == 0 || line.line_state_flag == 3) {
                    line.line_state_flag = 3;
                    if (abs_ir_angle <= -PI * 7.0 / 9.0 || abs_ir_angle > PI * 7.0 / 9.0) {
                        speed = 0;
                        motor_flag = 1;
                    } else if (abs_ir_angle > -PI * 7.0 / 9.0 && abs_ir_angle <= -PI / 2.0) {
                        speed = 0;
                        motor_flag = 1;
                    } else if (abs_ir_angle > -PI / 2.0 && abs_ir_angle < PI / 2.0) {
                        abs_move_angle = constrain(ir_angle, -PI / 4.0, PI / 4.0);
                        move_angle = abs_move_angle - gyro.angle;
                        speed = 100;
                        motor_flag = 0;
                    } else if (abs_ir_angle >= PI / 2.0 && abs_ir_angle <= PI * 7.0 / 9.0) {
                        speed = 0;
                        motor_flag = 1;
                    }
                } else {
                    esc_line();
                }
            }
        }
        if (!line.on_line) {           // ライン上にいない場合
            line.line_state_flag = 0;  // ラインの状態を初期化
            if (goal_flag) {           // ゴールがある場合．ゴールの角度に基づいて機体が向くべき角度を調整する．
                // この辺の処理はよくわからん（by根岸）．斎藤に聞いてくれ．
                if (goal_flag_ratio < 100) {
                    if (abs_goal_angle > 0) {
                        if (goal_flag_ratio <= 92) {
                            goal_flag_ratio += 8;
                        } else {
                            goal_flag_ratio = 100;
                        }
                    }
                    if (abs_goal_angle < 0) {
                        if (goal_flag_ratio <= 84) {
                            goal_flag_ratio += 16;
                        } else {
                            goal_flag_ratio = 100;
                        }
                    }
                }
                machine_angle = abs_goal_angle * GOAL_WEIGHT;
                if (machine_angle > -PI / 6 && gyro.angle < PI / 6) {
                    machine_angle += 0.7;
                    machine_angle *= 0.3;
                } else if (machine_angle < 0) {
                    machine_angle *= 0.8;
                }
                if (machine_angle > PI_THIRDS) {
                    machine_angle = PI_THIRDS;
                } else if (machine_angle < -PI_THIRDS) {
                    machine_angle = -PI_THIRDS;
                }
                machine_angle = normalize_angle(machine_angle);
            } else {
                // machine_angle = machine_angle * 0.9;
                if (machine_angle > 0.001) {
                    machine_angle -= 0.001;
                } else if (machine_angle < -0.001) {
                    machine_angle += 0.001;
                }
            }

            if (ir_flag) {                                 // ボールが検知された場合
                float circ_exp = pow(CIRC_BASE, ir_dist);  // 回り込みのための計算式．これがどうして導かれたかは謎．誰ももう覚えてない．
                // 計算で得られた角度が不適切な場合があるので，移動する方向を制限する．
                move_angle = ir_angle + constrain(ir_angle * circ_exp * CIRC_WEIGHT, -PI / 2.0, PI / 2.0);
                if (abs(move_angle) < PI / 6) {  // 移動する方向に応じて速度を変更する．
                    speed = STRAIGHT_SPEED;
                } else {
                    speed = CIRC_SPEED;
                }
            } else {  // ボールが検知されない場合．停止．
                move_angle = 0;
                speed = 0;
                motor_flag = 0;
            }
        }

        move_angle = normalize_angle(move_angle);  // 角度を-πからπまでに調整
        motor_uart_send();                         // モーター制御系の送信

        // ボタンの処理
        if (digitalRead(button_pin[0]) == LOW) {
            int cnt = 0;
            while (1) {
                if (digitalRead(button_pin[0]) == LOW) {
                    cnt++;
                    if (cnt == 100) {  // ボタンが十分な時間押されているかを確認（チャタリング対策）
                        break;
                    }
                } else {
                    cnt = 0;
                    break;
                }
                delay(2);
            }
            if (cnt == 100) {
                game_flag = 0;
                Serial.println("Game Stop");
            }
        }
        if (Serial.available()) {
            uint16_t volume = Serial.readStringUntil('\n').toInt();  // PCからのシリアル通信でモーターの速度を変更する（不要）.
            bldc_drive(volume);
        }
        end_time = micros();
        fps = 1000000.0 / (end_time - start_time);  // fpsの計算
    }
    if (digitalRead(button_pin[1]) == LOW) {
        int cnt = 0;
        while (1) {
            if (digitalRead(button_pin[1]) == LOW) {
                cnt++;
                if (cnt == 80) {  // ボタンが十分な時間押されているかを確認（チャタリング対策）
                    break;
                }
            } else {
                cnt = 0;
                break;
            }
            delay(2);
        }
        if (cnt == 80) {
            game_flag = 1;
            Serial.println("Game Start");
        }
    }

    // ゲームが始まっていない時
    move_angle = 0;
    speed = 0;
    motor_flag = 2;
    motor_uart_send();
}

// RP2024(マイコン)のコア1のsetupとloop
void setup1(void) {
}

void loop1(void) {
}

void motor_uart_send(void) {
    byte buf[8];
    buf[0] = constrain(motor_flag, 0, 254);    // 0: normal, 1: release, 2 or others: brake (0~254)
    uint16_t tmp = (move_angle + PI) * 100.0;  //-PI ~ PI -> 0 ~ 200PI
    buf[1] = tmp & 0b0000000001111111;         // 下位7bit
    buf[2] = tmp >> 7;                         // 上位2bit
    tmp = (gyro.angle + PI) * 100.0;           //-PI ~ PI -> 0 ~ 200PI
    buf[3] = tmp & 0b0000000001111111;         // 下位7bit
    buf[4] = tmp >> 7;                         // 上位3bit
    tmp = (machine_angle + PI) * 100.0;
    buf[5] = tmp & 0b0000000001111111;  // 下位7bit
    buf[6] = tmp >> 7;                  // 上位3bit
    buf[7] = constrain(speed, 0, 254);  // constrain(speed, 0, 254); // 0~254
    motor.write(255);                   // ヘッダー
    motor.write(buf, 8);
    motor.write(254);
    // Serial.println("Send");
}

void ir_uart_recv(void) {
    ir.write(255);  // ヘッダー
    unsigned long long request_time = micros();
    while (!ir.available()) {                   // データを受信するまで待つ
        if (micros() - request_time > 10000) {  // 10ms以上データが来ない場合は受信を諦める
            break;
        }
    }
    byte header = ir.read();  // ヘッダーを読み取る
    if (header != 255)
        return;  // ヘッダーが255でない場合は受信を諦める
    unsigned long long wait_time = micros();
    while (ir.available() < 4) {             // 4バイトのデータを受信するまで待つ
        if (micros() - wait_time > 10000) {  // 10ms以上データが来ない場合は受信を諦める
            while (ir.available()) {         // データが来ている場合は読み取る
                ir.read();
            }
            break;
        }
    }
    byte buf[4];  // データを格納するバッファ．データ構造については Xiao SAMD21 IRプロジェクトを参照
    ir.readBytes(buf, 4);
    if (buf[0] != 255 && buf[1] != 255 && buf[2] != 255 && buf[3] != 255) {
        ir_flag = buf[0];                                 // 0: normal, 1: stop (0~254)
        ir_angle = (buf[1] + buf[2] * 128) / 100.0 - PI;  // 0 ~ 200PI -> -PI ~ PI
        ir_dist = buf[3];                                 // 0~254[cm]
    }
}

void push_button(uint gpio, uint32_t events) {  // ボタンの処理（何してるか忘れた．解読する気もない．多分使ってない．）
    if (gpio == button_pin[0]) {
        gpio_set_irq_enabled(button_pin[0], GPIO_IRQ_EDGE_FALL, false);
        unsigned long long time1 = millis();
        while (digitalRead(button_pin[0]) == 0) {
            if (millis() - time1 > 1000) {
                break;
            }
        }
        // gyro.getEuler();
        // gyro.angle_offset = gyro.angle;
        game_flag = 1;
        // Serial.println("game start");
        gpio_set_irq_enabled(button_pin[0], GPIO_IRQ_EDGE_FALL, true);
    } else if (gpio == button_pin[1]) {
        gpio_set_irq_enabled(button_pin[1], GPIO_IRQ_EDGE_FALL, false);
        unsigned long long time1 = millis();
        while (digitalRead(button_pin[1]) == 0) {
            if (millis() - time1 > 1000) {
                break;
            }
        }
        if (!game_flag) {
            line_set_threshold_flag = 1;
        } else {
            gpio_set_irq_enabled(button_pin[1], GPIO_IRQ_EDGE_FALL, true);
        }
    } else if (gpio == button_pin[2]) {
        gpio_set_irq_enabled(button_pin[2], GPIO_IRQ_EDGE_FALL, false);
        unsigned long long time1 = millis();
        while (digitalRead(button_pin[2]) == 0) {
            if (millis() - time1 > 1000) {
                break;
            }
        }
        if (!line_set_threshold_flag) {
            game_flag = 0;
            // Serial.println("game stop");
            speed = 0;
            move_angle = 0;
            motor_flag = 1;
            motor_uart_send();
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("Waiting");
            display.display();
        }
        gpio_set_irq_enabled(button_pin[2], GPIO_IRQ_EDGE_FALL, true);
    }
}

void line_set_threshold() {  // ラインセンサーの閾値自動設定
    // 何しているかざっくりと解説．
    // ラインセンサーの閾値を自動設定する．
    // 一定時間の間，ロボットを白線と緑の部分を行ったり来たりさせる．その間，ラインセンサーの値を取得し，その平均値（多分）を閾値とする．
    Serial.println("Start auto threshold setting");
    Serial.println("Please push middle button to start");
    Serial.println("wait...");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print("Auto Set Threshold");
    display.setCursor(0, 8);
    display.print("Please Push Mid-Button");
    display.display();
    uint8_t cnt = 0;
    delay(1000);
    while (1) {
        if (digitalRead(button_pin[1]) == HIGH) {
            cnt = 0;
            delay(10);
        } else {
            cnt++;
            delay(10);
        }
        if (cnt > 5) {
            break;
        }
    }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Start");
    display.setCursor(0, 8);
    display.print("Move the Robot");
    display.display();
    delay(1000);
    // Serial.println("run");
    line.set_threshold();
    delay(1000);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Ended");
    display.setCursor(0, 8);
    display.print(line.ave_threshold);
    display.display();
    // Serial.println("End auto threshold setting");
    line_set_threshold_flag = false;
    gpio_set_irq_enabled(button_pin[1], GPIO_IRQ_EDGE_FALL, true);
}

void openmv_uart_recv(void) {  // OpenMVとの通信の受信
    if (Serial1.available()) {
        if (Serial1.read() == 255) {
            while (Serial1.available() < 3);
            uint8_t buf[3];
            buf[0] = Serial1.read();
            buf[1] = Serial1.read();
            buf[2] = Serial1.read();
            goal_angle = (buf[0] + buf[1] * 128.0) / 100.0 - PI;
            goal_flag = buf[2];

            if (abs(goal_angle) > PI) {
                // goal_flag = 0;
            }
            if (goal_flag) {
                abs_goal_angle = normalize_angle(goal_angle + gyro.angle);
            }
        }
    }
}

void bldc_init(void) {  // ブラシレスモータードライバーの初期化．Servoライブラリを使用して操作できるっぽい．
    esc.attach(ESC_PIN);
    esc.writeMicroseconds(MAX_SIGNAL);
    delay(2000);
    esc.writeMicroseconds(MIN_SIGNAL);
    delay(2000);
}

void bldc_drive(uint16_t volume) {  // ブラシレスモータードライバーの制御．Servoライブラリを使用して操作できるっぽい．
    esc.writeMicroseconds(volume);
}

double normalize_angle(double angle) {  // 角度を-πからπまでに調整する関数
    if (angle > 0) {
        angle = fmod(angle, TWO_PI);
        if (angle > PI) {
            angle = angle - TWO_PI;
        }
    } else if (angle < 0) {
        angle = fmod(abs(angle), TWO_PI);
        if (angle > PI) {
            angle = angle - TWO_PI;
        }
        angle = -angle;
    }
    return angle;
}

void esc_line(void) {  // ライン退避．ラインのが存在する角度の逆方向に進む．
    move_angle = line.line_theta + PI;
    if (move_angle > PI) {
        move_angle -= 2.0 * PI;
    }
    // move_angle = abs_move_angle - gyro.angle;
    speed = ESC_LINE_SPEED;
    motor_flag = 3;
}

void refresh_display(void) {  // ディスプレイのリフレッシュ．ディスプレイに表示する内容を更新する．
    display.clearDisplay();
    // ボールの位置を描画するための円と線を描画
    display.drawCircle(32, 32, 23, WHITE);
    display.drawLine(4, 32, 60, 32, WHITE);
    display.drawLine(32, 4, 32, 60, WHITE);
    int16_t ir_x, ir_y;  // 赤外線センサーの位置を計算
    ir_x = 32 + 23 * sin(ir_angle);
    ir_y = 32 - 23 * cos(ir_angle);
    display.fillCircle(ir_x, ir_y, 3, WHITE);  // 赤外線センサーの位置を描画
    // 機体の向きを計算
    int16_t gyro_x = 32 + 25 * sin(gyro.angle);
    int16_t gyro_y = 32 - 25 * cos(gyro.angle);
    display.drawLine(32, 32, gyro_x, gyro_y, WHITE);  // 機体の向きを描画
    // 様々な情報を表示
    display.setCursor(65, 0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print("IR:");
    display.print(ir_angle * 180 / PI, 1);  // 赤外線センサーの角度を表示
    display.setCursor(65, 8);
    display.print("GY:");
    display.print(gyro.angle * 180 / PI, 1);  // 6軸センサの角度を表示
    display.setCursor(65, 16);
    display.print("FPS:");
    display.print(fps);  // fpsを表示
    display.setCursor(65, 24);
    display.print("BAT:");
    display.print(battery_voltage, 1);  // バッテリーの電圧を表示
    display.setCursor(65, 32);
    display.print("L:");                               // ラインセンサーの情報を表示
    if (line.on_line) {                                // ライン上にいる場合
        display.print(line.line_theta * 180 / PI, 1);  // ラインの角度を表示
    } else {
        display.print("OFF");  // ライン上にいない場合
    }
    display.setCursor(65, 40);
    display.print("Desined By");
    display.setCursor(73, 48);
    display.setTextSize(2);
    display.print("Koji");
    display.display();
    display_refresh_time = millis();
}
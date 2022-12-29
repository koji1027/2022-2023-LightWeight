#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>

#include "MPU6050/gyro.h"
#include "led.h"
#include "line.h"

#define SCREEN_WIDTH 128        // OLED display width, in pixels
#define SCREEN_HEIGHT 64        // OLED display height, in pixels
#define SCREEN_REFRESH_TIME 50  // ms
#define LINE_FLAG_MAX 300
#define CIRCULATE_Kp 0.75

SerialPIO motor(22, 16, 32);
SerialPIO ir(1, 0, 32);
Line line;
Gyro gyro;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void display_begin();
void display_refresh_info();
void circulate();
int linetrace();

bool start_flag = false;
float ir_angle = 0.0;
int ir_radius = 0;
float circulate_angle = 0.0;
float linetrace_angle = 0.0;
int send_gyro = 0.0;
int send_move = 0.0;
int line_flag = 0;  // 左:1 右:2
int line_flag_count = 0;
bool line_emergency_flag = 0;
int line_emergency_count = 0;
int color[3] = {255, 255, 255};
int brightness = 255;
unsigned long long display_refresh_time = 0;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    gyro.begin();
    led.begin();
    line.begin();
    display_begin();
    display_refresh_time = millis();
    pinMode(D18, INPUT_PULLUP);
    pinMode(D19, INPUT_PULLUP);
    pinMode(D20, INPUT_PULLUP);
    pinMode(D21, INPUT_PULLUP);
    set_led(color, brightness);

    while (!start_flag) {
        if (!digitalRead(D20)) {
            line.set_threshold();
        }
        if (!digitalRead(D18)) {
            start_flag = true;
        }
    }
    Serial.println("Start!");
}

void loop() {
    while (start_flag) {
        gyro.getEuler();
        set_led(color, brightness);
        line.read();
        if (millis() - display_refresh_time > SCREEN_REFRESH_TIME) {
            display_refresh_time = millis();
            display_refresh_info();
        }
        if (!digitalRead(D18)) {
        }
        if (!digitalRead(D21)) {
            start_flag = false;
            Serial.println("Stop!");
        }
    }
}

void setup1() {
    motor.begin(115200);
    ir.begin(115200);
    byte data[5] = {0, 0, 0, 0, 2};
    motor.write(254);
    motor.write(254);
    motor.write(254);
    motor.write(254);
    motor.write(254);
    motor.write(254);
}

void loop1() {
    motor.write(254);
    motor.write(254);
    motor.write(254);
    motor.write(254);
    motor.write(254);
    motor.write(254);
    while (start_flag) {
        ir.write(255);
        if (ir.available() > 3) {
            int recv_data = ir.read();
            if (recv_data == 255) {
                int data[3];
                data[0] = ir.read();
                data[1] = ir.read();
                data[2] = ir.read();
                int a = data[0] + (data[1] << 8);
                ir_angle = (a / 100.0) - PI;
                ir_radius = data[2];
            }
        }
        /*circulate();

        send_gyro = (gyro.angle + PI) * 100;
        send_move = (circulate_angle + PI) * 100;
        int c = 0;  // 2:stop, 1:line, 0:ir
        c = linetrace();*/
        int c = 0;
        send_gyro = (gyro.angle + PI) * 100;
        float ball[2];
        ball[0] = ir_radius * sin(ir_angle);
        ball[1] = ir_radius * cos(ir_angle);
        float destination[2];
        float theta = 0;
        if (abs(ir_angle) < PI / 2.0 && abs(ir_angle) > PI / 9.0 &&
            ir_radius >= 10) {
            destination[0] = ball[0];
            destination[1] = ball[1] - 10;
            theta = atan2(destination[0], destination[1]);
        } else {
            c = 2;
        }
        Serial.print("ir:");
        Serial.print(ir_angle);
        Serial.print("\ttheta:");
        Serial.println(theta);

        send_move = (theta + PI) * 100;
        byte data[5];  //[0][1]:gyro, [2][3]:move
        data[0] = byte(send_gyro);
        data[1] = byte(send_gyro >> 8);
        data[2] = byte(send_move);
        data[3] = byte(send_move >> 8);
        data[4] = byte(c);
        motor.write(255);
        motor.write(data, 5);
    }
}

void circulate() {
    /*if (ir_angle > PI / 6) {
        circulate_angle = ir_angle + (PI / 3);
        if (circulate_angle > PI) {
            circulate_angle = circulate_angle - PI * 2;
        }
    } else if (ir_angle < -PI / 6) {
        circulate_angle = ir_angle - (PI / 3);
        if (circulate_angle < -PI) {
            circulate_angle = circulate_angle + PI * 2;
        }
    } else {
        circulate_angle = ir_angle;
    }*/
    circulate_angle = ir_angle + ir_angle * CIRCULATE_Kp;
    circulate_angle += PI * 10;
    circulate_angle = fmod(circulate_angle, PI * 2.0);
    if (circulate_angle > PI) {
        circulate_angle -= PI * 2.0;
    }
}

int linetrace() {
    // 現在、ラインがどこにあるかを判定　0:なし 1:前 2:後ろ 3:右 4:左
    // 5:その他
    int now_line_flag = 0;
    if (line.entire_sensor_state) {
        line_flag_count = 0;
        if (abs(line.line_theta) < PI / 6.0) {  // ロボの前側にラインあり
            now_line_flag = 1;
        } else if (line.line_theta > PI / 6.0 * 5.0 ||
                   line.line_theta <
                       -PI / 6.0 * 5.0) {  // ロボの後ろ側にラインあり
            now_line_flag = 2;
        } else if (line.line_theta > PI / 3.0 &&
                   line.line_theta <
                       PI / 3.0 * 2.0) {  // ロボの右側にラインあり
            now_line_flag = 3;
        } else if (line.line_theta < -PI / 3.0 &&
                   line.line_theta >
                       -PI / 3.0 * 2.0) {  // ロボの左側にラインあり
            now_line_flag = 4;
        } else {  // ラインを踏んでいるが左右ではない
            now_line_flag = 5;
        }
    } else {  // ラインを踏んでいない
        now_line_flag = 0;
    }

    if (line_flag == 0) {  // もともとラインを踏んでいないとき
        line_flag_count = 0;
        if (now_line_flag == 0) {  // 今も踏んでいない
            send_move = (circulate_angle + PI) * 100;  // 回り込みをする
            return 0;
        } else if (now_line_flag == 1) {  // 今は前にラインが有る
            line_flag = 1;
        } else if (now_line_flag == 2) {  // 今は後ろにラインが有る
            line_flag = 2;
        } else if (now_line_flag == 3) {  // 今は右にラインが有る
            line_flag = 3;
        } else if (line_flag == 4) {  // 今は左にラインが有る
            line_flag = 4;
        } else {  // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    } else if (line_flag == 1) {   // もともと前にラインが有るとき
        if (now_line_flag == 0) {  // 今はラインを踏んでいない
            if (line_flag_count < LINE_FLAG_MAX) {
                send_move = (circulate_angle + PI) * 100;  // 回り込みをする
                line_flag_count = 0;
                Serial.println("Reset");
                return 0;
            } else {
                line_flag_count++;
            }
        } else if (now_line_flag == 1) {  // 前を踏んでいる
            line_flag = 1;
        } else if (now_line_flag == 2) {  // 後ろを踏んでいる
            line_emergency_flag = 1;
            send_move = (PI + PI) * 100;  // 後ろに下がってもとに戻る
            return 0;
        } else if (now_line_flag == 3) {  // 右を踏んでいる
            line_flag = 3;
        } else if (now_line_flag == 4) {  // 左を踏んでいる
            line_flag = 4;
        } else {  // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    } else if (line_flag == 2) {  // もともと後ろにラインが有るとき
        if (now_line_flag == 0) {  // 今はラインを踏んでいない
            if (line_flag_count < LINE_FLAG_MAX) {
                send_move = (circulate_angle + PI) * 100;  // 回り込みをする
                line_flag_count = 0;
                Serial.println("Reset");
                return 0;
            } else {
                line_flag_count++;
            }
        } else if (now_line_flag == 1) {  // 前を踏んでいる
            line_emergency_flag = 1;
            send_move = (0 + PI) * 100;  // 前に進んでもとに戻る
            return 0;
        } else if (now_line_flag == 2) {  // 後ろを踏んでいる
            line_flag = 2;
        } else if (now_line_flag == 3) {  // 右を踏んでいる
            line_flag = 3;
        } else if (now_line_flag == 4) {  // 左を踏んでいる
            line_flag = 4;
        } else {  // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    } else if (line_flag == 3) {   // もともと右にラインが有るとき
        if (now_line_flag == 0) {  // 今はラインを踏んでいない
            if (line_flag_count < LINE_FLAG_MAX) {
                send_move = (circulate_angle + PI) * 100;  // 回り込みをする
                line_flag_count = 0;
                Serial.println("Reset");
                return 0;
            } else {
                line_flag_count++;
            }
        } else if (now_line_flag == 1) {  // 前を踏んでいる
            line_flag = 1;
        } else if (now_line_flag == 2) {  // 後ろを踏んでいる
            line_flag = 2;
        } else if (now_line_flag == 3) {  // 右を踏んでいる
            line_flag = 3;
        } else if (now_line_flag == 4) {  // 左を踏んでいる
            line_emergency_flag = 1;
            send_move = (-PI / 2.0 + PI) * 100;  // 左に進んでもとに戻る
            return 0;
        } else {  // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    } else if (line_flag == 4) {   // もともと左にラインが有るとき
        if (now_line_flag == 0) {  // 今はラインを踏んでいない
            if (line_flag_count < LINE_FLAG_MAX) {
                send_move = (circulate_angle + PI) * 100;  // 回り込みをする
                line_flag_count = 0;
                Serial.println("Reset");
                return 0;
            } else {
                line_flag_count++;
            }
        } else if (now_line_flag == 1) {  // 前を踏んでいる
            line_flag = 1;
        } else if (now_line_flag == 2) {  // 後ろを踏んでいる
            line_flag = 2;
        } else if (now_line_flag == 3) {  // 右を踏んでいる
            line_emergency_flag = 1;
            send_move = (PI / 2.0 + PI) * 100;  // 右に進んでもとに戻る
            return 0;
        } else if (now_line_flag == 4) {  // 左を踏んでいる
            line_flag = 4;
        } else {  // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    } else {  // もともとラインを踏んでいるが前後左右ではない
        if (now_line_flag == 0) {  // 今はラインを踏んでいない
            if (line_flag_count < LINE_FLAG_MAX) {
                send_move = (circulate_angle + PI) * 100;  // 回り込みをする
                line_flag_count = 0;
                Serial.println("Reset");
                return 0;
            } else {
                line_flag_count++;
            }
        } else if (now_line_flag == 1) {  // 前を踏んでいる
            line_flag = 1;
        } else if (now_line_flag == 2) {  // 後ろを踏んでいる
            line_flag = 2;
        } else if (now_line_flag == 3) {  // 右を踏んでいる
            line_flag = 3;
        } else if (now_line_flag == 4) {  // 左を踏んでいる
            line_flag = 4;
        } else {  // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
    if (line_flag == 0) {                          // ラインを分でない
        send_move = (circulate_angle + PI) * 100;  // 回り込みをする
        return 0;
    } else if (line_flag == 1) {          // 前を踏んでいる
        if (abs(ir_angle) <= PI / 6.0) {  // ボールが前にある
            return 2;                     // 静止
        } else if (ir_angle > PI / 6.0 &&
                   ir_angle < PI / 2.0) {  // ボールが右前にある
            send_move = (PI / 12.0 * 7.0 + PI) * 100;  // 右に動く
            return 0;
        } else if (ir_angle < -PI / 6.0 &&
                   ir_angle > -PI / 2.0) {  // ボールが左前にある
            send_move = (-PI / 12.0 * 7.0 + PI) * 100;  // 左に動く
            return 0;
        } else {  // ボールが後ろにある
            send_move = (circulate_angle + PI) * 100;  // 回り込み
            return 0;
        }
    } else if (line_flag == 2) {  // 後ろを踏んでいる
        if (abs(ir_angle) <= PI / 3.0) {
            send_move = (circulate_angle + PI) * 100;
            return 0;
        } else if (ir_angle > PI / 3.0 && ir_angle <= PI / 9.0 * 8.0) {
            send_move = (PI / 2.0 + PI) * 100;
            return 0;
        } else if (ir_angle < -PI / 3.0 && ir_angle >= -PI / 9.0 * 8.0) {
            send_move = (-PI / 2.0 + PI) * 100;
            return 0;
        } else {
            return 2;
        }
        return 0;
    } else if (line_flag == 3) {  // 右を踏んでいる
        if (ir_angle >= 0 && ir_angle <= PI / 9.0 * 4.0) {
            send_move = (0 + PI) * 100;
            return 0;
        } else if (ir_angle > PI / 9.0 * 4.0 && ir_angle < PI / 9.0 * 5.0) {
            return 2;
        } else if (ir_angle >= PI / 9.0 * 5.0) {
            send_move = (PI + PI) * 100;
            return 0;
        } else if (ir_angle < 0 && ir_angle >= -PI / 3.0 * 2.0) {
            send_move = (circulate_angle + PI) * 100;
            return 0;
        } else {
            send_move = ((ir_angle + PI / 3.0) + PI) * 100;
            return 0;
        }
    } else if (line_flag == 4) {  // 左を踏んでいる
        if (ir_angle <= 0 && ir_angle >= -PI / 9.0 * 4.0) {
            send_move = (0 + PI) * 100;
            return 0;
        } else if (ir_angle < -PI / 9.0 * 4.0 && ir_angle > -PI / 9.0 * 5.0) {
            return 2;
        } else if (ir_angle <= -PI / 9.0 * 5.0) {
            send_move = (PI + PI) * 100;
            return 0;
        } else if (ir_angle > 0 && ir_angle <= PI / 3.0 * 2.0) {
            send_move = (circulate_angle + PI) * 100;
            return 0;
        } else {
            send_move = ((ir_angle - PI / 3.0) + PI) * 100;
            return 0;
        }
    } else {
        send_move = (line.line_theta + PI) * 100;
        return 1;
    }
}

void display_begin() {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextColor(SSD1306_WHITE);
}

void display_refresh_info() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(64, 0);
    display.print("Gyro: ");
    display.setTextSize(2);
    display.setCursor(64, 8);
    int gyro_deg = gyro.angle * 180.0 / PI;
    display.println(gyro_deg);
    display.setTextSize(1);
    display.setCursor(64, 24);
    display.print("Line: ");
    display.setTextSize(2);
    display.setCursor(64, 32);
    if (line.entire_sensor_state) {
        int line_deg = line.line_theta * 180.0 / PI;
        display.println(line_deg);
    } else {
        display.println("N/A");
    }
    display.drawCircle(32, 32, 15, WHITE);
    display.drawLine(32, 13, 32, 51, WHITE);
    display.drawLine(13, 32, 51, 32, WHITE);
    float ball[2] = {0, 0};
    ball[0] = 25 * sin(ir_angle) + 32;
    ball[1] = -25 * cos(ir_angle) + 32;
    display.fillCircle(ball[0], ball[1], 3, WHITE);
    display.display();
}
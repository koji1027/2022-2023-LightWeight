#include <Adafruit_GFX.h>
#include <Arduino.h>
#include <Wire.h>
#include <bmx055.h>

#include "Adafruit_SSD1306.h"

#define SERIAL_BAUD 115200

Axis bmx055;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

const String mode[1] = {"Game"};

bool line_flag = false;
float ball_dir = 0.00;
int ball_dist = 150;
int current_mode = 0;

void display_init();                               // OLED display init
void display_ball(float ball_dir, int ball_dist);  // draw ball
void display_line(bool line_flag);                 // draw line
void display_other_info(int current_mode);         // draw other info

/* OLED display の使い方
    初期化
        1, Adafruit_SSD1306 display(横px数, 縦px数, &Wire, -1)
        2, Setup内で、display_init()を呼び出す

    文字の表示
        1, display.clearDisplay(); で画面をクリア
        2, display.setTextSize(文字サイズ); で文字サイズを設定
        3, display.setTextColor(文字色); で文字色を設定
        4, display.setCursor(文字のx座標, 文字のy座標); で文字の位置を設定
        5, display.println(表示したい文字列); で文字列を表示
            ※display.print()でも可
        6, display.display(); で画面に表示

    ボール情報の表示
        1, display_ball(ボールの方向, ボールからの距離); でボールを描画
            ※ボールの方向は度数法
            ※ボールからの距離は３桁まで表示可能

    ライン情報の表示
        1, display_line(ラインの有無); でラインを描画
            ※ラインの有無はbool型

    その他情報の表示
        1, display_other_info(); でその他情報を描画
        ※モード、クレジットの表示

    注意事項　ー＞　前回と表示内容が異なる場合にのみ描画するようにしてください。

*/

void setup() {
    // put your setup code here, to run once:
    Serial.begin(SERIAL_BAUD);
    Serial1.begin(SERIAL_BAUD);
    Wire.begin();
    bmx055.init();
    bmx055.show(false, false, false);
}

void setup1() {
    Serial1.begin(SERIAL_BAUD);
    pinMode(A0, INPUT);
    analogReadResolution(10);
    display_init();
}

void loop() {
    bmx055.cal();
    bmx055.show(0, 0, 0);
}

void loop1() {
    if (Serial1.available() > 0) {
        int recv_data = Serial1.read();
        if (recv_data == 255) {
            float battery_voltage = analogRead(A0);
            battery_voltage = battery_voltage * 3.3 / 1023.0;
            battery_voltage *= 4.0;
            bool battery_voltage_flag =
                (battery_voltage < 10.0 && battery_voltage > 6.0);
            bmx055.send(battery_voltage_flag);
        } else if (recv_data == 254) {
            bmx055.gyro_reset();
        }
    }
}

void display_init() {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.display();
    /*display.setTextSize(1);
    display.setTextColor(WHITE);
    display.drawCircle(32, 32, 12, WHITE);
    display.writeLine(17, 32, 47, 32, WHITE);
    display.writeLine(32, 17, 32, 47, WHITE);
    display.display();*/
}

void display_ball(float ball_deg, int ball_dist) {
    ball_deg = fmod(ball_deg, 360.0);
    float ball_rad = ball_deg / 180.0 * PI;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.drawCircle(32, 32, 12, WHITE);
    display.writeLine(17, 32, 47, 32, WHITE);
    display.writeLine(32, 17, 32, 47, WHITE);
    float ball_x = round(sin(ball_rad) * 27 + 32);
    float ball_y = round(32 - cos(ball_rad) * 27);
    display.fillCircle(ball_x, ball_y, 3, WHITE);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(64, 0);
    display.println("Ball:");
    display.setCursor(80, 8);
    float theta = round(ball_rad / PI * 100) / 100.0;
    if (theta > 1) {
        theta -= 2;
    }
    display.print(theta);
    display.println("PI");
    display.setCursor(64, 16);
    display.print("Dist:");
    display.print(ball_dist);
    display.println("cm");
}

void display_line(bool line_flag) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(64, 28);
    if (line_flag) {
        display.println("On Line");
    } else {
        display.println("No Line");
    }
}

void display_other_info(int current_mode) {
    display.setCursor(64, 40);
    display.print("Mode:");
    display.println(mode[current_mode]);
    display.setTextSize(1);
    display.setCursor(64, 52);
    display.print("MadeByKoji");
}
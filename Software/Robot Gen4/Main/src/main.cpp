#include <Arduino.h>
#include <hardware/flash.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "MPU6050/gyro.h"
#include "led.h"
#include "line.h"

#define DIST_BALL -20.0
#define CIRC_BASE pow(0.6, 1.0 / 20.0)
#define CIRC_WEIGHT 3.5
#define IS_LINE_GOAL 50
#define IS_CORNER_GOAL 20
#define CIRCULATE_SPEED 100
#define STRAIGHT_SPEED 180
#define GOAL_LPF 0.1

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

SerialPIO motor(D17, D16, 32);
SerialPIO ir(D0, D1, 32);
Gyro gyro;
Line line;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int BUTTON_PIN[3] = {D18, D19, D20};

bool start_flag = false;
float machine_angle = 0.0;
float ir_angle = 0.0;
float absolute_ir_angle = 0.0;
float ir_radius = 0.0;
float move_angle = 0.0;
float absolute_move_angle = 0.0;
int ball_flag = 0; // 0:なし 1:あり
int line_flag = 0;
int led_color[3] = {255, 255, 255};
int led_brightness = 50;
int volt = 0;
int is_line_count = 0;
bool is_corner = false;
int is_corner_count = 0;
float corner_dir = 0.0;
int line_emergency_flag = 0;
float circulate_angle = 0.0;
double goal_angle = 0.0;
double goal_angle_LPF = 0.0;
bool goal_flag = 0;

float vx = 0.0;
float vy = 0.0; // 機体から見た相対的な向き
int speed = 0;

void line_trace_vertical();
void line_trace();
static void save_setting_to_flash();
void load_setting_from_flash();
void monitor_update();

void setup()
{
    pinMode(BUTTON_PIN[0], INPUT_PULLUP);
    pinMode(BUTTON_PIN[1], INPUT_PULLUP);
    pinMode(BUTTON_PIN[2], INPUT_PULLUP);
    Serial.begin(115200);
    Serial1.setTX(D12);
    Serial1.setRX(D13);
    Serial1.begin(115200);
    load_setting_from_flash();
    gyro.begin();
    line.begin();
    init_led();
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    display.display();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    delay(1000);
}

void loop()
{ // Serial.println(digitalRead(BUTTON_PIN[0]));
  // Serial.println(start_flag);
    while (start_flag)
    {
        gyro.getEuler();
        line.read();
        // Serial.println(gyro.angle);
        // line.print();
        volt = analogRead(A3);
        line.absolute_line_theta = line.line_theta + gyro.angle;
        if (line.absolute_line_theta > PI)
        {
            line.absolute_line_theta -= 2 * PI;
        }
        if (line.absolute_line_theta < -PI)
        {
            line.absolute_line_theta += 2 * PI;
        }
        if (digitalRead(BUTTON_PIN[2]) == LOW)
        {
            start_flag = false;
        }
        // Serial.println(line.is_line);
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
                Serial.println(goal_angle);
                // Serial.println(goal_angle / PI * 180.0);
            }
        }
        monitor_update();
    }

    if (digitalRead(BUTTON_PIN[0]) == LOW)
    {
        start_flag = true;
        gyro.getEuler();
        gyro.angle_offset = gyro.angle;
    }
    /*
    if (digitalRead(BUTTON_PIN[1]) == LOW)
    {
        line.set_threshold();
        delay(1000);
    }
    */
    display.setCursor(75, 50);
    display.print(volt);
    display.display();
    
    delay(10);
}

void setup1()
{
    motor.begin(115200);
    ir.begin(115200);
    delay(3000);
}

void loop1()
{
    while (start_flag)
    {
        ir.write(255);
        if (ir.available() > 5)
        {
            int recv_data = ir.read();
            if (recv_data == 255)
            {
                byte data[4];
                data[0] = ir.read();
                data[1] = ir.read();
                data[2] = ir.read();
                data[3] = ir.read();
                float _ir_angle = (float)(data[0] + (data[1] << 8)) / 100.0 - PI;
                if (abs(_ir_angle) <= PI)
                {
                    ir_angle = _ir_angle;
                }
                absolute_ir_angle = ir_angle + gyro.angle;
                if (absolute_ir_angle > PI)
                {
                    absolute_ir_angle -= 2 * PI;
                }
                if (absolute_ir_angle < -PI)
                {
                    absolute_ir_angle += 2 * PI;
                }
                ir_radius = (float)data[2];
                ball_flag = data[3];
            }
        }
        // Serial.println(ir_angle);
        // Serial.println(ir_radius);
        // Serial.println(gyro.angle);
        // ball_flag = false;
        if (ball_flag)
        {
            float circ_exp = pow(CIRC_BASE, ir_radius);
            if (abs(absolute_ir_angle) < PI / 2)
            {
                absolute_move_angle = absolute_ir_angle + constrain(absolute_ir_angle * circ_exp * CIRC_WEIGHT, -PI / 2, PI / 2);
                speed = CIRCULATE_SPEED;
            }
            else
            {
                absolute_move_angle = PI;
                speed = STRAIGHT_SPEED;
            }
            move_angle = absolute_move_angle - machine_angle;
            vx = cos(move_angle);
            vy = sin(move_angle);

            // machine_angle = absolute_ir_angle * circ_exp;
        }
        else
        {
            vx = 0;
            vy = 0;
            speed = 0;
        }

        goal_angle = goal_angle - 0.1;
        // Serial.println(goal_angle);
        if (abs(goal_angle) < PI * 2 / 3)
        {
            goal_flag = 1;
        }
        else
        {
            goal_flag = 0;
        }

        if (goal_flag)
        {
            goal_angle_LPF = goal_angle * GOAL_LPF + goal_angle_LPF * (1 - GOAL_LPF);
            machine_angle = goal_angle_LPF * 2.3;
        }
        else
        {
            machine_angle = 0.0;
            // goal_angle_LPF = 0.0;
        }

        /*
        Serial.print(line.cluster_num);
        Serial.print("\t");
        Serial.println(line.line_theta);
        */

        // line_trace_vertical();

        // ↓ラインは出ないけどガクガクする単純な処理

        // Serial.println(line.is_line);
        if (line.is_line)
        {
            vx = cos(line.line_theta + PI);
            vy = sin(line.line_theta + PI);
        }

        vx = vx / sqrt(vx * vx + vy * vy);
        vy = vy / sqrt(vx * vx + vy * vy);
        float _vx = (vx + 1.0) * 100.0;
        float _vy = (vy + 1.0) * 100.0;
        byte data[8];
        data[0] = _vx; // 送るもとの値は -1~1 だが、送るときは 0~200 にする
        data[1] = _vy;
        data[2] = speed;
        data[3] = (byte)((gyro.angle + PI) * 100);
        data[4] = (byte)((int)((gyro.angle + PI) * 100) >> 8);
        data[5] = (byte)((machine_angle + PI) * 100);
        data[6] = (byte)((int)((machine_angle + PI) * 100) >> 8);
        data[7] = 0;
        motor.write(255);
        motor.write(data, 8);
        delay(10);
    }
    byte data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 1;
    motor.write(255);
    motor.write(data, 8);
    delay(10);
}

void line_trace_vertical()
{
    // 現在、ラインがどこにあるかを判定　0:なし 1:前 2:後ろ 3:右 4:左 5:その他
    int now_line_flag = 0;
    if (line.is_line)
    {
        is_line_count = 0;
        if (abs(line.line_theta) < PI / 6.0)
        { // ロボの前側にラインあり
            now_line_flag = 1;
        }
        else if (line.line_theta > PI / 6.0 * 5.0 || line.line_theta < -PI / 6.0 * 5.0)
        { // ロボの後ろ側にラインあり
            now_line_flag = 2;
        }
        else if (line.line_theta > PI / 3.0 && line.line_theta < PI / 3.0 * 2.0)
        { // ロボの右側にラインあり
            now_line_flag = 3;
        }
        else if (line.line_theta < -PI / 3.0 && line.line_theta > -PI / 3.0 * 2.0)
        { // ロボの左側にラインあり
            now_line_flag = 4;
        }
        else
        { // ラインを踏んでいるが左右ではない
            now_line_flag = 5;
        }
    }
    else
    { // ラインを踏んでいない
        now_line_flag = 0;
    }

    if (line_flag == 0)
    { // もともとラインを踏んでいないとき
        is_line_count = 0;
        if (now_line_flag == 0)
        {
        } // 今も踏んでいない
        else if (now_line_flag == 1)
        { // 今は前にラインが有る
            line_flag = 1;
        }
        else if (now_line_flag == 2)
        { // 今は後ろにラインが有る
            line_flag = 2;
        }
        else if (now_line_flag == 3)
        { // 今は右にラインが有る
            line_flag = 3;
        }
        else if (line_flag == 4)
        { // 今は左にラインが有る
            line_flag = 4;
        }
        else
        { // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
    else if (line_flag == 1)
    { // もともと前にラインが有るとき
        vx = cos(line.line_theta + PI);
        vy = sin(line.line_theta + PI);
        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            is_line_count++;
            if (is_line_count >= IS_LINE_GOAL)
            {
                line_flag = 0;
                is_line_count = 0;
                // Serial.println("Reset");
            }
        }
        else if (now_line_flag == 1)
        { // 前を踏んでいる
            line_flag = 1;
        }
        else if (now_line_flag == 2)
        { // 後ろを踏んでいる
            line_emergency_flag = 1;
            vx = -1;
        }
        else if (now_line_flag == 3)
        { // 右を踏んでいる
            line_flag = 3;
        }
        else if (now_line_flag == 4)
        { // 左を踏んでいる
            line_flag = 4;
        }
        else
        { // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
    else if (line_flag == 2)
    { // もともと後ろにラインが有るとき
        vx = cos(line.line_theta + PI);
        vy = sin(line.line_theta + PI);
        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (is_line_count >= IS_LINE_GOAL)
            {
                line_flag = 0;
                is_line_count = 0;
                // Serial.println("Reset");
            }
            else
            {
                is_line_count++;
            }
        }
        else if (now_line_flag == 1)
        { // 前を踏んでいる
            line_emergency_flag = 1;
            vx = 1;
        }
        else if (now_line_flag == 2)
        { // 後ろを踏んでいる
            line_flag = 2;
        }
        else if (now_line_flag == 3)
        { // 右を踏んでいる
            line_flag = 3;
        }
        else if (now_line_flag == 4)
        { // 左を踏んでいる
            line_flag = 4;
        }
        else
        { // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
    else if (line_flag == 3)
    { // もともと右にラインが有るとき
        if (vy > 0)
        {
            vy = 0;
        }
        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (is_line_count >= IS_LINE_GOAL)
            {
                line_flag = 0;
                is_line_count = 0;
                // Serial.println("Reset");
            }
            else
            {
                is_line_count++;
            }
        }
        else if (now_line_flag == 1)
        { // 前を踏んでいる
            line_flag = 1;
        }
        else if (now_line_flag == 2)
        { // 後ろを踏んでいる
            line_flag = 2;
        }
        else if (now_line_flag == 3)
        { // 右を踏んでいる
            line_flag = 3;
        }
        else if (now_line_flag == 4)
        { // 左を踏んでいる
            line_emergency_flag = 1;
            vy = -1;
        }
        else
        { // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
    else if (line_flag == 4)
    { // もともと左にラインが有るとき
        if (vy < 0)
        {
            vy = 0;
        }
        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (is_line_count >= IS_LINE_GOAL)
            {
                line_flag = 0;
                is_line_count = 0;
                // Serial.println("Reset");
            }
            else
            {
                is_line_count++;
            }
        }
        else if (now_line_flag == 1)
        { // 前を踏んでいる
            line_flag = 1;
        }
        else if (now_line_flag == 2)
        { // 後ろを踏んでいる
            line_flag = 2;
        }
        else if (now_line_flag == 3)
        { // 右を踏んでいる
            line_emergency_flag = 1;
            vy = 1;
        }
        else if (now_line_flag == 4)
        { // 左を踏んでいる
            line_flag = 4;
        }
        else
        { // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
    else
    { // もともとラインを踏んでいるが前後左右ではない

        vx = cos(line.line_theta + PI);
        vy = sin(line.line_theta + PI);

        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (is_line_count >= IS_LINE_GOAL)
            {
                line_flag = 0;
                is_line_count = 0;
                // Serial.println("Reset");
            }
            else
            {
                is_line_count++;
            }
        }
        else if (now_line_flag == 1)
        { // 前を踏んでいる
            line_flag = 1;
        }
        else if (now_line_flag == 2)
        { // 後ろを踏んでいる
            line_flag = 2;
        }
        else if (now_line_flag == 3)
        { // 右を踏んでいる
            line_flag = 3;
        }
        else if (now_line_flag == 4)
        { // 左を踏んでいる
            line_flag = 4;
        }
        else
        { // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
}

void line_trace()
{ // 現在、ラインがどこにあるかを判定　0:なし 1:前 2:後ろ 3:右 4:左 5:その他
    int now_line_flag = 0;
    if (line.is_line)
    {
        is_line_count = 0;
        if (abs(line.line_theta) < PI / 6.0)
        { // ロボの前側にラインあり
            now_line_flag = 1;
        }
        else if (line.line_theta > PI / 6.0 * 5.0 || line.line_theta < -PI / 6.0 * 5.0)
        { // ロボの後ろ側にラインあり
            now_line_flag = 2;
        }
        else if (line.line_theta > PI / 3.0 && line.line_theta < PI / 3.0 * 2.0)
        { // ロボの右側にラインあり
            now_line_flag = 3;
        }
        else if (line.line_theta < -PI / 3.0 && line.line_theta > -PI / 3.0 * 2.0)
        { // ロボの左側にラインあり
            now_line_flag = 4;
        }
        else
        { // ラインを踏んでいるが左右ではない
            now_line_flag = 5;
        }
    }
    else
    { // ラインを踏んでいない
        now_line_flag = 0;
    }

    if (is_corner == false)
    {
        if (line_flag == 0)
        { // もともとラインを踏んでいないとき
            is_line_count = 0;
            if (now_line_flag == 0)
            {
            } // 今も踏んでいない
            else if (now_line_flag == 1)
            { // 今は前にラインが有る
                line_flag = 1;
            }
            else if (now_line_flag == 2)
            { // 今は後ろにラインが有る
                line_flag = 2;
            }
            else if (now_line_flag == 3)
            { // 今は右にラインが有る
                line_flag = 3;
            }
            else if (line_flag == 4)
            { // 今は左にラインが有る
                line_flag = 4;
            }
            else
            { // ラインを踏んでいるが前後左右ではない
                line_flag = 5;
            }
        }
        else if (line_flag == 1)
        { // もともと前にラインが有るとき
            if (vx > 0)
            {
                vx = 0;
            }
            if (now_line_flag == 0)
            { // 今はラインを踏んでいない
                is_line_count++;
                if (is_line_count >= IS_LINE_GOAL)
                {
                    line_flag = 0;
                    is_line_count = 0;
                    // Serial.println("Reset");
                }
            }
            else if (now_line_flag == 1)
            { // 前を踏んでいる
                line_flag = 1;
            }
            else if (now_line_flag == 2)
            { // 後ろを踏んでいる
                line_emergency_flag = 1;
                vx = -1;
            }
            else if (now_line_flag == 3)
            { // 右を踏んでいる
                line_flag = 3;
            }
            else if (now_line_flag == 4)
            { // 左を踏んでいる
                line_flag = 4;
            }
            else
            { // ラインを踏んでいるが前後左右ではない
                line_flag = 5;
            }
        }
        else if (line_flag == 2)
        { // もともと後ろにラインが有るとき
            if (vx < 0)
            {
                vx = 0;
            }
            if (now_line_flag == 0)
            { // 今はラインを踏んでいない
                if (is_line_count >= IS_LINE_GOAL)
                {
                    line_flag = 0;
                    is_line_count = 0;
                    // Serial.println("Reset");
                }
                else
                {
                    is_line_count++;
                }
            }
            else if (now_line_flag == 1)
            { // 前を踏んでいる
                line_emergency_flag = 1;
                vx = 1;
            }
            else if (now_line_flag == 2)
            { // 後ろを踏んでいる
                line_flag = 2;
            }
            else if (now_line_flag == 3)
            { // 右を踏んでいる
                line_flag = 3;
            }
            else if (now_line_flag == 4)
            { // 左を踏んでいる
                line_flag = 4;
            }
            else
            { // ラインを踏んでいるが前後左右ではない
                line_flag = 5;
            }
        }
        else if (line_flag == 3)
        { // もともと右にラインが有るとき
            if (vy > 0)
            {
                vy = 0;
            }
            if (now_line_flag == 0)
            { // 今はラインを踏んでいない
                if (is_line_count >= IS_LINE_GOAL)
                {
                    line_flag = 0;
                    is_line_count = 0;
                    // Serial.println("Reset");
                }
                else
                {
                    is_line_count++;
                }
            }
            else if (now_line_flag == 1)
            { // 前を踏んでいる
                line_flag = 1;
            }
            else if (now_line_flag == 2)
            { // 後ろを踏んでいる
                line_flag = 2;
            }
            else if (now_line_flag == 3)
            { // 右を踏んでいる
                line_flag = 3;
            }
            else if (now_line_flag == 4)
            { // 左を踏んでいる
                line_emergency_flag = 1;
                vy = -1;
            }
            else
            { // ラインを踏んでいるが前後左右ではない
                line_flag = 5;
            }
        }
        else if (line_flag == 4)
        { // もともと左にラインが有るとき
            if (vy < 0)
            {
                vy = 0;
            }
            if (now_line_flag == 0)
            { // 今はラインを踏んでいない
                if (is_line_count >= IS_LINE_GOAL)
                {
                    line_flag = 0;
                    is_line_count = 0;
                    // Serial.println("Reset");
                }
                else
                {
                    is_line_count++;
                }
            }
            else if (now_line_flag == 1)
            { // 前を踏んでいる
                line_flag = 1;
            }
            else if (now_line_flag == 2)
            { // 後ろを踏んでいる
                line_flag = 2;
            }
            else if (now_line_flag == 3)
            { // 右を踏んでいる
                line_emergency_flag = 1;
                vy = 1;
            }
            else if (now_line_flag == 4)
            { // 左を踏んでいる
                line_flag = 4;
            }
            else
            { // ラインを踏んでいるが前後左右ではない
                line_flag = 5;
            }
        }
        else
        { // もともとラインを踏んでいるが前後左右ではない

            vx = cos(line.line_theta + PI) / 2;
            vy = sin(line.line_theta + PI) / 2;

            /*
            if(abs(line.line_theta) > PI/2){
                vx = -1;
                vy = 0;
            } else{
                vx = 1;
                vy = 0;
            }
            */
            if (now_line_flag == 0)
            { // 今はラインを踏んでいない
                if (is_line_count >= IS_LINE_GOAL)
                {
                    line_flag = 0;
                    is_line_count = 0;
                    // Serial.println("Reset");
                }
                else
                {
                    is_line_count++;
                }
            }
            else if (now_line_flag == 1)
            { // 前を踏んでいる
                line_flag = 1;
            }
            else if (now_line_flag == 2)
            { // 後ろを踏んでいる
                line_flag = 2;
            }
            else if (now_line_flag == 3)
            { // 右を踏んでいる
                line_flag = 3;
            }
            else if (now_line_flag == 4)
            { // 左を踏んでいる
                line_flag = 4;
            }
            else
            { // ラインを踏んでいるが前後左右ではない
                line_flag = 5;
            }
        }
        // Serial.println(atan2(vy,vx)/PI);
        if (line.cluster_num >= 2)
        {
            is_corner = true;
            corner_dir = line.line_theta;
        }
    }
    else
    {
        if (corner_dir > 0 && corner_dir < PI / 2)
        {
            vx = -1;
            vy = -1;
        }
        else if (corner_dir > PI / 2 && corner_dir < PI)
        {
            vx = 1;
            vy = -1;
        }
        else if (corner_dir > -PI && corner_dir < -PI / 2)
        {
            vx = 1;
            vy = 1;
        }

        else if (corner_dir > -PI / 2 && corner_dir < 0)
        {
            vx = -1;
            vy = 1;
        }
        is_corner_count++;
        if (is_corner_count >= IS_CORNER_GOAL)
        {
            is_corner = false;
            is_corner_count = 0;
            corner_dir = 0;
        }
    }
}

/*
void line_trace()
{ // 現在、ラインがどこにあるかを判定　0:なし 1:前 2:後ろ 3:右 4:左 5:その他
    int now_line_flag = 0;
    if (line.is_line)
    {
        is_line_count = 0;
        if (abs(line.line_theta) < PI / 6.0)
        { // ロボの前側にラインあり
            now_line_flag = 1;
        }
        else if (line.line_theta > PI / 6.0 * 5.0 || line.line_theta < -PI / 6.0 * 5.0)
        { // ロボの後ろ側にラインあり
            now_line_flag = 2;
        }
        else if (line.line_theta > PI / 3.0 && line.line_theta < PI / 3.0 * 2.0)
        { // ロボの右側にラインあり
            now_line_flag = 3;
        }
        else if (line.line_theta < -PI / 3.0 && line.line_theta > -PI / 3.0 * 2.0)
        { // ロボの左側にラインあり
            now_line_flag = 4;
        }
        else
        { // ラインを踏んでいるが左右ではない
            now_line_flag = 5;
        }
    }
    else
    { // ラインを踏んでいない
        now_line_flag = 0;
    }

    if (is_corner == false)
    {
        if (line_flag == 0)
        { // もともとラインを踏んでいないとき
            is_line_count = 0;
            if (now_line_flag == 0)
            {
            } // 今も踏んでいない
            else if (now_line_flag == 1)
            { // 今は前にラインが有る
                line_flag = 1;
            }
            else if (now_line_flag == 2)
            { // 今は後ろにラインが有る
                line_flag = 2;
            }
            else if (now_line_flag == 3)
            { // 今は右にラインが有る
                line_flag = 3;
            }
            else if (line_flag == 4)
            { // 今は左にラインが有る
                line_flag = 4;
            }
            else
            { // ラインを踏んでいるが前後左右ではない
                line_flag = 5;
            }
        }
        else if (line_flag == 1)
        { // もともと前にラインが有るとき
            if (vx > 0)
            {
                vx = 0;
            }
            if (now_line_flag == 0)
            { // 今はラインを踏んでいない
                is_line_count++;
                if (is_line_count >= IS_LINE_GOAL)
                {
                    line_flag = 0;
                    is_line_count = 0;
                    // Serial.println("Reset");
                }
            }
            else if (now_line_flag == 1)
            { // 前を踏んでいる
                line_flag = 1;
            }
            else if (now_line_flag == 2)
            { // 後ろを踏んでいる
                line_emergency_flag = 1;
                vx = -1;
            }
            else if (now_line_flag == 3)
            { // 右を踏んでいる
                line_flag = 3;
            }
            else if (now_line_flag == 4)
            { // 左を踏んでいる
                line_flag = 4;
            }
            else
            { // ラインを踏んでいるが前後左右ではない
                line_flag = 5;
            }
        }
        else if (line_flag == 2)
        { // もともと後ろにラインが有るとき
            if (vx < 0)
            {
                vx = 0;
            }
            if (now_line_flag == 0)
            { // 今はラインを踏んでいない
                if (is_line_count >= IS_LINE_GOAL)
                {
                    line_flag = 0;
                    is_line_count = 0;
                    // Serial.println("Reset");
                }
                else
                {
                    is_line_count++;
                }
            }
            else if (now_line_flag == 1)
            { // 前を踏んでいる
                line_emergency_flag = 1;
                vx = 1;
            }
            else if (now_line_flag == 2)
            { // 後ろを踏んでいる
                line_flag = 2;
            }
            else if (now_line_flag == 3)
            { // 右を踏んでいる
                line_flag = 3;
            }
            else if (now_line_flag == 4)
            { // 左を踏んでいる
                line_flag = 4;
            }
            else
            { // ラインを踏んでいるが前後左右ではない
                line_flag = 5;
            }
        }
        else if (line_flag == 3)
        { // もともと右にラインが有るとき
            if (vy > 0)
            {
                vy = 0;
            }
            if (now_line_flag == 0)
            { // 今はラインを踏んでいない
                if (is_line_count >= IS_LINE_GOAL)
                {
                    line_flag = 0;
                    is_line_count = 0;
                    // Serial.println("Reset");
                }
                else
                {
                    is_line_count++;
                }
            }
            else if (now_line_flag == 1)
            { // 前を踏んでいる
                line_flag = 1;
            }
            else if (now_line_flag == 2)
            { // 後ろを踏んでいる
                line_flag = 2;
            }
            else if (now_line_flag == 3)
            { // 右を踏んでいる
                line_flag = 3;
            }
            else if (now_line_flag == 4)
            { // 左を踏んでいる
                line_emergency_flag = 1;
                vy = -1;
            }
            else
            { // ラインを踏んでいるが前後左右ではない
                line_flag = 5;
            }
        }
        else if (line_flag == 4)
        { // もともと左にラインが有るとき
            if (vy < 0)
            {
                vy = 0;
            }
            if (now_line_flag == 0)
            { // 今はラインを踏んでいない
                if (is_line_count >= IS_LINE_GOAL)
                {
                    line_flag = 0;
                    is_line_count = 0;
                    // Serial.println("Reset");
                }
                else
                {
                    is_line_count++;
                }
            }
            else if (now_line_flag == 1)
            { // 前を踏んでいる
                line_flag = 1;
            }
            else if (now_line_flag == 2)
            { // 後ろを踏んでいる
                line_flag = 2;
            }
            else if (now_line_flag == 3)
            { // 右を踏んでいる
                line_emergency_flag = 1;
                vy = 1;
            }
            else if (now_line_flag == 4)
            { // 左を踏んでいる
                line_flag = 4;
            }
            else
            { // ラインを踏んでいるが前後左右ではない
                line_flag = 5;
            }
        }
        else
        { // もともとラインを踏んでいるが前後左右ではない

            vx = cos(line.line_theta + PI)/2;
            vy = sin(line.line_theta + PI)/2;


            if (now_line_flag == 0)
            { // 今はラインを踏んでいない
                if (is_line_count >= IS_LINE_GOAL)
                {
                    line_flag = 0;
                    is_line_count = 0;
                    // Serial.println("Reset");
                }
                else
                {
                    is_line_count++;
                }
            }
            else if (now_line_flag == 1)
            { // 前を踏んでいる
                line_flag = 1;
            }
            else if (now_line_flag == 2)
            { // 後ろを踏んでいる
                line_flag = 2;
            }
            else if (now_line_flag == 3)
            { // 右を踏んでいる
                line_flag = 3;
            }
            else if (now_line_flag == 4)
            { // 左を踏んでいる
                line_flag = 4;
            }
            else
            { // ラインを踏んでいるが前後左右ではない
                line_flag = 5;
            }
        }
        // Serial.println(atan2(vy,vx)/PI);
        if (line.cluster_num == 2)
        {
            is_corner = true;
        }
    }
    else{
        vx = cos(line.line_theta + PI);
        vy = sin(line.line_theta + PI);
        is_corner_count++;
        if(is_corner_count >= IS_CORNER_GOAL){
            is_corner = false;
            is_corner_count = 0;
        }
    }
}
*/

/*
void line_trace()
{   // 現在、ラインがどこにあるかを判定　0:なし 1:前 2:後ろ 3:右 4:左 5:その他
    int now_line_flag = 0;
    int pre_line_flag = 0;
    if (line.is_line)
    {
        no_line_count = 0;
        if (abs(line.line_theta) < PI / 4.0)
        { // ロボの前側にラインあり
            now_line_flag = 1;
        }
        else if (line.line_theta > PI / 4.0 * 3.0 || line.line_theta < -PI / 4.0 * 3.0)
        { // ロボの後ろ側にラインあり
            now_line_flag = 2;
        }
        else if (line.line_theta > PI / 4.0 && line.line_theta < PI / 4.0 * 3.0)
        { // ロボの右側にラインあり
            now_line_flag = 3;
        }
        else if (line.line_theta < -PI / 4.0 && line.line_theta > -PI / 4.0 * 3.0)
        { // ロボの左側にラインあり
            now_line_flag = 4;
        }
        else
        { // ラインを踏んでいるが左右ではない
            now_line_flag = 5;
        }
    }
    else
    { // ラインを踏んでいない
        now_line_flag = 0;
    }

    if (line_flag == 0)
    { // もともとラインを踏んでいないとき
        no_line_count = 0;
        if (now_line_flag == 0){}// 今も踏んでいない
        else if (now_line_flag == 1)
        { // 今は前にラインが有る
            line_flag = 1;
        }
        else if (now_line_flag == 2)
        { // 今は後ろにラインが有る
            line_flag = 2;
        }
        else if (now_line_flag == 3)
        { // 今は右にラインが有る
            line_flag = 3;
        }
        else if (line_flag == 4)
        { // 今は左にラインが有る
            line_flag = 4;
        }
        else
        { // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
    else if (line_flag == 1)
    { // もともと前にラインが有るとき
        if(vx > 0){
            vx = 0;
        }
        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (pre_line_flag == line_flag){
                count_available = true;
            }
            if (count_available){
                no_line_count++;
                if (no_line_count >= NO_LINE_GOAL)
                {
                    line_flag = 0;
                    no_line_count = 0;
                    count_available = false;
                    //Serial.println("Reset");
                }
            }
            else {
                vx = -1;
                escape_flag = true;
            }

        }
        else if (now_line_flag == 1)
        { // 前を踏んでいる
            line_flag = 1;
            if (escape_flag){
                count_available = true;
                escape_flag = false;
            }
        }
        else if (now_line_flag == 2)
        { // 後ろを踏んでいる
            count_available = false;
            vx = -1;
        }
        else if (now_line_flag == 3)
        { // 右を踏んでいる
            line_flag = 3;
        }
        else if (now_line_flag == 4)
        { // 左を踏んでいる
            line_flag = 4;
        }
        else
        { // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
    else if (line_flag == 2)
    { // もともと後ろにラインが有るとき
        if(vx < 0){
            vx = 0;
        }
        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (pre_line_flag == line_flag){
                count_available = true;
            }
            if (count_available){
                no_line_count++;
                if (no_line_count >= NO_LINE_GOAL)
                {
                    line_flag = 0;
                    no_line_count = 0;
                    count_available = false;
                    //Serial.println("Reset");
                }
            }
            else {
                vx = 1;
                escape_flag = true;
            }
        }
        else if (now_line_flag == 1)
        { // 前を踏んでいる
            count_available = false;
            vx = 1;
        }
        else if (now_line_flag == 2)
        { // 後ろを踏んでいる
            line_flag = 2;
            if (escape_flag){
                count_available = true;
                escape_flag = false;
            }
        }
        else if (now_line_flag == 3)
        { // 右を踏んでいる
            line_flag = 3;
        }
        else if (now_line_flag == 4)
        { // 左を踏んでいる
            line_flag = 4;
        }
        else
        { // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
    else if (line_flag == 3)
    { // もともと右にラインが有るとき
        if(vy > 0){
            vy = 0;
        }
        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (pre_line_flag == line_flag){
                count_available = true;
            }
            if (count_available){
                no_line_count++;
                if (no_line_count >= NO_LINE_GOAL)
                {
                    line_flag = 0;
                    no_line_count = 0;
                    count_available = false;
                    //Serial.println("Reset");
                }
            }
            else {
                vy = -1;
                escape_flag = true;
            }
        }
        else if (now_line_flag == 1)
        { // 前を踏んでいる
            line_flag = 1;
        }
        else if (now_line_flag == 2)
        { // 後ろを踏んでいる
            line_flag = 2;
        }
        else if (now_line_flag == 3)
        { // 右を踏んでいる
            line_flag = 3;
            if (escape_flag){
                count_available = true;
                escape_flag = false;
            }
        }
        else if (now_line_flag == 4)
        { // 左を踏んでいる
            count_available = false;
            vy = -1;
        }
        else
        { // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
    else if (line_flag == 4)
    { // もともと左にラインが有るとき
        if(vy < 0){
            vy = 0;
        }
        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (pre_line_flag == line_flag){
                count_available = true;
            }
            if (count_available){
                no_line_count++;
                if (no_line_count >= NO_LINE_GOAL)
                {
                    line_flag = 0;
                    no_line_count = 0;
                    count_available = false;
                    //Serial.println("Reset");
                }
            }
            else {
                vy = 1;
                escape_flag = true;
            }
        }
        else if (now_line_flag == 1)
        { // 前を踏んでいる
            line_flag = 1;
        }
        else if (now_line_flag == 2)
        { // 後ろを踏んでいる
            line_flag = 2;
        }
        else if (now_line_flag == 3)
        { // 右を踏んでいる
            count_available = false;
            vy = 1;
        }
        else if (now_line_flag == 4)
        { // 左を踏んでいる
            line_flag = 4;
            if (escape_flag){
                count_available = true;
                escape_flag = false;
            }
        }
        else
        { // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
    else
    { // もともとラインを踏んでいるが前後左右ではない

        vx = cos(line.line_theta + PI);
        vy = sin(line.line_theta + PI);

        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (no_line_count >= NO_LINE_GOAL)
            {
                line_flag = 0;
                no_line_count = 0;
                //Serial.println("Reset");
            }
            else
            {
                no_line_count++;
            }
        }
        else if (now_line_flag == 1)
        { // 前を踏んでいる
            line_flag = 1;
        }
        else if (now_line_flag == 2)
        { // 後ろを踏んでいる
            line_flag = 2;
        }
        else if (now_line_flag == 3)
        { // 右を踏んでいる
            line_flag = 3;
        }
        else if (now_line_flag == 4)
        { // 左を踏んでいる
            line_flag = 4;
        }
        else
        { // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
    //Serial.println(atan2(vy,vx)/PI);
    if(line.cluster_num == 2){
        if(abs(line.line_theta) > PI/2){
            vx = -1;
            vy = 0;
        } else{
            vx = 1;
            vy = 0;
        }
    }
    pre_line_flag = now_line_flag;
}*/

void monitor_update()
{
    display.clearDisplay();
    display.drawCircle(SCREEN_WIDTH / 4, SCREEN_HEIGHT / 2, 28, SSD1306_WHITE);
    if (ball_flag)
    {
        display.fillCircle(SCREEN_WIDTH / 4 + cos(absolute_ir_angle) * 28, SCREEN_HEIGHT / 2 + sin(absolute_ir_angle) * 28, 4, SSD1306_WHITE);
    }
    display.drawLine(SCREEN_WIDTH / 4, SCREEN_HEIGHT / 2,
                     SCREEN_WIDTH / 4 + cos(absolute_move_angle) * 24, SCREEN_HEIGHT / 2 + sin(absolute_move_angle) * 24, SSD1306_WHITE);
    display.drawLine(SCREEN_WIDTH / 4 + cos(absolute_move_angle) * 24, SCREEN_HEIGHT / 2 + sin(absolute_move_angle) * 24,
                     (SCREEN_WIDTH / 4 + cos(absolute_move_angle) * 24) + cos(absolute_move_angle - PI + PI / 6) * 8,
                     (SCREEN_HEIGHT / 2 + sin(absolute_move_angle) * 24) + sin(absolute_move_angle - PI + PI / 6) * 8, SSD1306_WHITE);
    display.drawLine(SCREEN_WIDTH / 4 + cos(absolute_move_angle) * 24, SCREEN_HEIGHT / 2 + sin(absolute_move_angle) * 24,
                     (SCREEN_WIDTH / 4 + cos(absolute_move_angle) * 24) + cos(absolute_move_angle - PI - PI / 6) * 8,
                     (SCREEN_HEIGHT / 2 + sin(absolute_move_angle) * 24) + sin(absolute_move_angle - PI - PI / 6) * 8, SSD1306_WHITE);
    if (line.is_line)
    {
        display.setCursor(75, 5);
        display.println("LINE");
    }
    if (goal_flag)
    {
        display.setCursor(75, 27);
        display.println("GOAL");
    }
    display.setCursor(75, 50);
    display.print(volt);
    display.display();
};

static void save_setting_to_flash()
{
    const uint32_t FLASH_TARGET_OFFSET = 0x1F0000;
    uint8_t write_data[FLASH_PAGE_SIZE];
    // write_dataに書き込みたいデータを格納

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_PAGE_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, write_data, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

void load_setting_from_flash()
{
    const uint32_t FLASH_TARGET_OFFSET = 0x1F0000;
    const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    for (int i = 0; i < 6; i++)
    {
        uint8_t low = flash_target_contents[i * 2];
        uint8_t high = flash_target_contents[i * 2 + 1];
        int value = (high << 8) + low;
        if (value >= 16384)
        {
            value -= 32768;
        }
        gyro.offset[i] = value;
    }
}
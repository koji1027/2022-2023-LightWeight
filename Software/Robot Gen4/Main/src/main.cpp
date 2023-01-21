#include <Arduino.h>

#include "MPU6050/gyro.h"
#include "led.h"
#include "line.h"

#define DIST_BALL -20.0
#define CIRC_BASE pow(0.75, 1.0 / 20.0)
#define LINE_FLAG_MAX 100

SerialPIO motor(D17, D16, 32);
SerialPIO ir(D0, D1, 32);
Gyro gyro;
Line line;

const int BUTTON_PIN[3] = {D18, D19, D20};

bool start_flag = false;
float machine_angle = 0.0;
float ir_angle = 0.0;
float ir_radius = 0.0;
float move_angle = 0.0;
int ball_flag = 0; // 0:なし 1:あり
int line_flag = 0;
int led_color[3] = {255, 255, 255};
int led_brightness = 50;
int line_flag_count = 0;
int line_emergency_flag = 0;
float circulate_angle = 0.0;

int linetrace();

void setup()
{
    pinMode(BUTTON_PIN[0], INPUT_PULLUP);
    pinMode(BUTTON_PIN[1], INPUT_PULLUP);
    pinMode(BUTTON_PIN[2], INPUT_PULLUP);
    Serial.begin(115200);
    gyro.begin();
    line.begin();
    init_led();
    delay(1000);
}

void loop()
{
    while (start_flag)
    {
        gyro.getEuler();
        line.read();
        Serial.println(gyro.angle);
    }
    if (digitalRead(BUTTON_PIN[0]) == LOW)
    {
        start_flag = true;
    }
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
                ir_radius = (float)data[2];
                ball_flag = data[3];
            }
        }
        if (ball_flag)
        {
            float Circ_Kp = pow(CIRC_BASE, ir_radius);
            circulate_angle = ir_angle + ir_angle * Circ_Kp;
            move_angle = circulate_angle;
            // float line_flag = linetrace();
            float vx = sin(move_angle);
            float vy = cos(move_angle);

            /* ミスター齊藤のライン制作領域
            if (line.line_flag) {
                if ((line.line_theta >= 0 && line.line_theta <= PI / 4) ||
                    (line.line_theta > -PI / 4 && line.line_theta <= 0)) {
                    vy = 0;
                }
                if (line.line_theta > PI / 4 && line.line_theta <= 3 * PI / 4) {
                    vx = 0;
                }
                if ((line.line_theta > 3 * PI / 4 && line.line_theta <= PI) ||
                    (line.line_theta >= -PI && line.line_theta <= -3 * PI / 4))
            { vy = 0;
                }
                if (line.line_theta > -3 * PI / 4 && line.line_theta >= -PI / 4)
            { vx = 0;
                }
            }

            if (line.line_flag){vx=0; vy=0;}
            */

            vx = (vx + 1.0) * 100.0;
            vy = (vy + 1.0) * 100.0;

            byte data[8];
            data[0] = vx; // 送るもとの値は -1~1 だが、送るときは 0~200 にする
            data[1] = vy;
            data[2] = 100;
            data[3] = (byte)((gyro.angle + PI) * 100);
            data[4] = (byte)((int)((gyro.angle + PI) * 100) >> 8);
            data[5] = (byte)((machine_angle + PI) * 100);
            data[6] = (byte)((int)((machine_angle + PI) * 100) >> 8);
            data[7] = 0;

            motor.write(255);
            motor.write(data, 8);
            delay(10);
        }
        else
        {
            byte data[8];
            data[0] = 0; // 送るもとの値は -1~1 だが、送るときは 0~200 にする
            data[1] = 0;
            data[2] = 0;
            data[3] = (byte)((gyro.angle + PI) * 100);
            data[4] = (byte)((int)((gyro.angle + PI) * 100) >> 8);
            data[5] = (byte)((machine_angle + PI) * 100);
            data[6] = (byte)((int)((machine_angle + PI) * 100) >> 8);
            data[7] = 0;

            motor.write(255);
            motor.write(data, 8);
            delay(10);
        }
    }
}

int linetrace()
{
    // 現在、ラインがどこにあるかを判定　0:なし 1:前 2:後ろ 3:右 4:左
    // 5:その他
    int now_line_flag = 0;
    if (line.line_flag)
    {
        line_flag_count = 0;
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
        line_flag_count = 0;
        if (now_line_flag == 0)
        {                                              // 今も踏んでいない
            move_angle = (circulate_angle + PI) * 100; // 回り込みをする
            return 0;
        }
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
        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (line_flag_count < LINE_FLAG_MAX)
            {
                move_angle = (circulate_angle + PI) * 100; // 回り込みをする
                line_flag_count = 0;
                Serial.println("Reset");
                return 0;
            }
            else
            {
                line_flag_count++;
            }
        }
        else if (now_line_flag == 1)
        { // 前を踏んでいる
            line_flag = 1;
        }
        else if (now_line_flag == 2)
        { // 後ろを踏んでいる
            line_emergency_flag = 1;
            move_angle = (PI + PI) * 100; // 後ろに下がってもとに戻る
            return 0;
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
        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (line_flag_count < LINE_FLAG_MAX)
            {
                move_angle = (circulate_angle + PI) * 100; // 回り込みをする
                line_flag_count = 0;
                Serial.println("Reset");
                return 0;
            }
            else
            {
                line_flag_count++;
            }
        }
        else if (now_line_flag == 1)
        { // 前を踏んでいる
            line_emergency_flag = 1;
            move_angle = (0 + PI) * 100; // 前に進んでもとに戻る
            return 0;
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
        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (line_flag_count < LINE_FLAG_MAX)
            {
                move_angle = (circulate_angle + PI) * 100; // 回り込みをする
                line_flag_count = 0;
                Serial.println("Reset");
                return 0;
            }
            else
            {
                line_flag_count++;
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
            move_angle = (-PI / 2.0 + PI) * 100; // 左に進んでもとに戻る
            return 0;
        }
        else
        { // ラインを踏んでいるが前後左右ではない
            line_flag = 5;
        }
    }
    else if (line_flag == 4)
    { // もともと左にラインが有るとき
        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (line_flag_count < LINE_FLAG_MAX)
            {
                move_angle = (circulate_angle + PI) * 100; // 回り込みをする
                line_flag_count = 0;
                Serial.println("Reset");
                return 0;
            }
            else
            {
                line_flag_count++;
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
            move_angle = (PI / 2.0 + PI) * 100; // 右に進んでもとに戻る
            return 0;
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
        if (now_line_flag == 0)
        { // 今はラインを踏んでいない
            if (line_flag_count < LINE_FLAG_MAX)
            {
                move_angle = (circulate_angle + PI) * 100; // 回り込みをする
                line_flag_count = 0;
                Serial.println("Reset");
                return 0;
            }
            else
            {
                line_flag_count++;
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
    if (line_flag == 0)
    {                                              // ラインを分でない
        move_angle = (circulate_angle + PI) * 100; // 回り込みをする
        return 0;
    }
    else if (line_flag == 1)
    { // 前を踏んでいる
        if (abs(ir_angle) <= PI / 6.0)
        {             // ボールが前にある
            return 2; // 静止
        }
        else if (ir_angle > PI / 6.0 && ir_angle < PI / 2.0)
        {                                              // ボールが右前にある
            move_angle = (PI / 12.0 * 7.0 + PI) * 100; // 右に動く
            return 0;
        }
        else if (ir_angle < -PI / 6.0 && ir_angle > -PI / 2.0)
        {                                               // ボールが左前にある
            move_angle = (-PI / 12.0 * 7.0 + PI) * 100; // 左に動く
            return 0;
        }
        else
        {                                              // ボールが後ろにある
            move_angle = (circulate_angle + PI) * 100; // 回り込み
            return 0;
        }
    }
    else if (line_flag == 2)
    { // 後ろを踏んでいる
        if (abs(ir_angle) <= PI / 3.0)
        {
            move_angle = (circulate_angle + PI) * 100;
            return 0;
        }
        else if (ir_angle > PI / 3.0 && ir_angle <= PI / 9.0 * 8.0)
        {
            move_angle = (PI / 2.0 + PI) * 100;
            return 0;
        }
        else if (ir_angle < -PI / 3.0 && ir_angle >= -PI / 9.0 * 8.0)
        {
            move_angle = (-PI / 2.0 + PI) * 100;
            return 0;
        }
        else
        {
            return 2;
        }
        return 0;
    }
    else if (line_flag == 3)
    { // 右を踏んでいる
        if (ir_angle >= 0 && ir_angle <= PI / 9.0 * 4.0)
        {
            move_angle = (0 + PI) * 100;
            return 0;
        }
        else if (ir_angle > PI / 9.0 * 4.0 && ir_angle < PI / 9.0 * 5.0)
        {
            return 2;
        }
        else if (ir_angle >= PI / 9.0 * 5.0)
        {
            move_angle = (PI + PI) * 100;
            return 0;
        }
        else if (ir_angle < 0 && ir_angle >= -PI / 3.0 * 2.0)
        {
            move_angle = (circulate_angle + PI) * 100;
            return 0;
        }
        else
        {
            move_angle = ((ir_angle + PI / 3.0) + PI) * 100;
            return 0;
        }
    }
    else if (line_flag == 4)
    { // 左を踏んでいる
        if (ir_angle <= 0 && ir_angle >= -PI / 9.0 * 4.0)
        {
            move_angle = (0 + PI) * 100;
            return 0;
        }
        else if (ir_angle < -PI / 9.0 * 4.0 && ir_angle > -PI / 9.0 * 5.0)
        {
            return 2;
        }
        else if (ir_angle <= -PI / 9.0 * 5.0)
        {
            move_angle = (PI + PI) * 100;
            return 0;
        }
        else if (ir_angle > 0 && ir_angle <= PI / 3.0 * 2.0)
        {
            move_angle = (circulate_angle + PI) * 100;
            return 0;
        }
        else
        {
            move_angle = ((ir_angle - PI / 3.0) + PI) * 100;
            return 0;
        }
    }
    else
    {
        move_angle = (line.line_theta + PI) * 100;
        return 1;
    }
}
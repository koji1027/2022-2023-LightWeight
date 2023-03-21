#include <Arduino.h>
#include <hardware/flash.h>

#include "MPU6050/gyro.h"
#include "led.h"
#include "line.h"

#define DIST_BALL -20.0
#define CIRC_BASE pow(0.8, 1.0 / 20.0)
#define LINE_FLAG_MAX 100
#define KEEPER_SPEED_Kp 3.0
#define KEEPER_X_Kp 7.5

SerialPIO motor(D17, D16, 32);
SerialPIO esp32(D12, D13, 32);
Gyro gyro;
Line line;

const int BUTTON_PIN[3] = {D18, D19, D20};

bool start_flag = false;
float machine_angle = 0.0;
float ir_angle = 0.0;
float absolute_ir_angle = 0.0;
float ir_radius = 0.0;
float move_angle = 0.0;
int ball_flag = 0; // 0:なし 1:あり
int line_flag = 0; // 0:前 1:後ろ
int led_color[3] = {255, 255, 255};
int led_brightness = 50;
int line_flag_count = 0;
int line_emergency_flag = 0;
float circulate_angle = 0.0;
int distance[3];

float vx = 0.0;
float vy = 0.0;
int default_speed = 100;
int speed = 0;

int linetrace();
static void save_setting_to_flash();
void load_setting_from_flash();

void setup()
{
    pinMode(BUTTON_PIN[0], INPUT_PULLUP);
    pinMode(BUTTON_PIN[1], INPUT_PULLUP);
    pinMode(BUTTON_PIN[2], INPUT_PULLUP);
    Serial.begin(115200);
    esp32.begin(115200);
    while (!esp32)
        ;

    load_setting_from_flash();
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
        line.absolute_line_theta = line.line_theta + gyro.angle;
        if (line.absolute_line_theta > PI)
        {
            line.absolute_line_theta -= 2 * PI;
        }
        if (line.absolute_line_theta < -PI)
        {
            line.absolute_line_theta += 2 * PI;
        }
        if (esp32.available() > 3)
        {
            int recv_data = esp32.read();
            if (recv_data == 255)
            {
                byte data[3];
                for (int i = 0; i < 3; i++)
                {
                    data[i] = esp32.read();
                }
                for (int i = 0; i < 3; i++)
                {
                    distance[i] = distance[i] * 0.15 + data[i] * 0.85;
                    Serial.print(distance[i]);
                    Serial.print("\t");
                }
                Serial.println();
            }
        }
        if (digitalRead(BUTTON_PIN[2]) == LOW)
        {
            start_flag = false;
        }
    }
    if (digitalRead(BUTTON_PIN[0]) == LOW)
    {
        start_flag = true;
        gyro.getEuler();
        gyro.angle_offset = gyro.angle;
    }
    if (digitalRead(BUTTON_PIN[1]) == LOW)
    {
        line.set_threshold();
        delay(1000);
    }
    delay(10);
}

void setup1()
{
    motor.begin(115200);
    Serial1.begin(115200);
    delay(4000);
}

void loop1()
{
    while (start_flag)
    {
        Serial1.write(255);
        if (Serial1.available() > 4)
        {
            int recv_data = Serial1.read();
            if (recv_data == 255)
            {
                byte data[4];
                data[0] = Serial1.read();
                data[1] = Serial1.read();
                data[2] = Serial1.read();
                data[3] = Serial1.read();
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
        if (ball_flag)
        {
            /*if (distance[0] < 50)
            {
                vx = 0;
                vy = -1;
                speed = default_speed;
                Serial.println("Right");
            }
            else if (distance[2] < 50)
            {
                vx = 0;
                vy = 1;
                speed = default_speed;
                Serial.println("Left");
            }
            else
            {
                if (!line.line_flag)
                {
                    if (line_flag)
                    {
                        if (abs(ir_angle) < PI / 6.0)
                        {
                            vx = -1;
                            vy = 0;
                            speed = default_speed;
                        }
                        else if (ir_angle >= PI / 6.0 && ir_angle <= PI / 2.0)
                        {
                            vx = -sqrt(3) / 2;
                            vy = 0.5;
                            speed = default_speed;
                        }
                        else if (ir_angle <= -PI / 6.0 && ir_angle >= -PI / 2.0)
                        {
                            vx = -sqrt(3) / 2;
                            vy = -0.5;
                            speed = default_speed;
                        }
                    }
                    else
                    {
                        if (abs(ir_angle) < PI / 6.0)
                        {
                            vx = 1;
                            vy = 0;
                            speed = default_speed;
                        }
                        else if (ir_angle >= PI / 6.0 && ir_angle <= PI / 2.0)
                        {
                            vx = sqrt(3) / 2;
                            vy = 0.5;
                            speed = default_speed;
                        }
                        else if (ir_angle <= -PI / 6.0 && ir_angle >= -PI / 2.0)
                        {
                            vx = sqrt(3) / 2;
                            vy = -0.5;
                            speed = default_speed;
                        }
                    }
                }
                else
                {
                    if (abs(line.line_theta) < PI / 2.0)
                    {
                        line_flag = 0;
                    }
                    else
                    {
                        line_flag = 1;
                    }
                    if (abs(ir_angle) < PI / 6.0)
                    {
                        vx = 0;
                        vy = 0;
                        speed = 0;
                    }
                    else if (ir_angle >= PI / 6.0 && ir_angle <= PI / 2.0)
                    {
                        vx = 0;
                        vy = 1;
                        speed = default_speed;
                    }
                    else if (ir_angle <= -PI / 6.0 && ir_angle >= -PI / 2.0)
                    {
                        vx = 0;
                        vy = -1;
                        speed = default_speed;
                    }
                    else
                    {
                        vx = 0;
                        vy = 0;
                        speed = 0;
                    }
                }
            }*/
            vx = 0;
            vy = 0;
            speed = default_speed;
            if (distance[0] < 58)
            {
                vy = -1;
            }
            if (distance[2] < 58)
            {
                vy = 1;
            }
            else
            {
                ir_angle += PI / 12.0;
                if (ir_angle > PI)
                {
                    ir_angle -= 2 * PI;
                }
                if (ir_angle > 0)
                {
                    vx = 0;
                    vy = 1;
                    speed = (ir_angle / PI * 180.0) * KEEPER_SPEED_Kp * 1.25;
                }
                else if (ir_angle < 0)
                {
                    vx = 0;
                    vy = -1;
                    speed = -(ir_angle / PI * 180.0) * KEEPER_SPEED_Kp * 1.25;
                }
                else
                {
                    vx = 0;
                    vy = 0;
                    speed = 0;
                }
                speed = constrain(speed, 0, 100);
            }

            vx = (float)(22 - distance[1]) / KEEPER_X_Kp;
            vx = constrain(vx, -1, 1);

            if (vy == 0)
            {
                speed = KEEPER_SPEED_Kp * abs(25 - distance[1]) * 2.0;
                vx /= abs(vx);
            }
            else
            {
                vx = vx / sqrt(vx * vx + vy * vy);
                vy = vy / sqrt(vx * vx + vy * vy);
            }
        }
        else
        {
            vx = 0;
            if (distance[0] - distance[2] > 0)
            {
                vy = 1;
            }
            else
            {
                vy = -1;
            }
            speed = KEEPER_SPEED_Kp * abs(distance[0] - distance[2]) * 2.0;
            speed = constrain(speed, 0, 100);
        }

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
    data[0] = 100;
    data[1] = 100;
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
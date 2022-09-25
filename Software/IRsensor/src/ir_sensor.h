#include <Arduino.h>
#include <moving_average.h>

#ifndef __IR_SENSOR__
#define __IR_SENSOR__

#define IR_NUM 16
#define IR_CUR_MAX 1023
#define s0 D2
#define s1 D3
#define s2 D4
#define s3 D5
#define SIG_pin A1
MovingAverage ave(20);

class IR
{
public:
    void begin();
    int read(int channel);
    void print();
    float get_radius();
    float get_angle();
    int IR_Cur[IR_NUM];

private:
    int muxChannel[16][4] = {
        {0, 0, 0, 0}, // channel 0
        {1, 0, 0, 0}, // channel 1
        {0, 1, 0, 0}, // channel 2
        {1, 1, 0, 0}, // channel 3
        {0, 0, 1, 0}, // channel 4
        {1, 0, 1, 0}, // channel 5
        {0, 1, 1, 0}, // channel 6
        {1, 1, 1, 0}, // channel 7
        {0, 0, 0, 1}, // channel 8
        {1, 0, 0, 1}, // channel 9
        {0, 1, 0, 1}, // channel 10
        {1, 1, 0, 1}, // channel 11
        {0, 0, 1, 1}, // channel 12
        {1, 0, 1, 1}, // channel 13
        {0, 1, 1, 1}, // channel 14
        {1, 1, 1, 1}  // channel 15
    };

    typedef struct
    {
        int active_num; // 反応したセンサの個数
        int max_val;    // 最大のセンサ値
        int max_index;  // 最大の値を観測したセンサの番号
    } sensorInfo_t;
    typedef struct
    {
        float x;
        float y;
    } vectorXY_t;

    typedef struct
    {
        float radius;
        float angle;
    } vectorRT_t;

    sensorInfo_t IR_Info;
    vectorXY_t vector_XY;
    vectorRT_t vector_RT;
    // vectorXY_t vector_XY_close;
    // vectorRT_t vector_RT_close;

    // float unit_radius = 9;
    // float boal_radius = 3.6;
    // float R = boal_radius + unit_radius;
    // int MAX_R = 1600;
    float IR_IN[IR_NUM] = {0, -PI / 8, -PI * 2 / 8, -PI * 3 / 8, -PI * 4 / 8, -PI * 5 / 8, -PI * 6 / 8, -PI * 7 / 8, PI, PI * 7 / 8, PI * 6 / 8, PI * 5 / 8, PI * 4 / 8, PI * 3 / 8, PI * 2 / 8, PI / 8}; //ピンの角度
    float IR_cor[IR_NUM] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    float unit_angle[IR_NUM];
    float unit_sin[IR_NUM];
    float unit_cos[IR_NUM];
    // float unit_cor[IR_NUM];
    float now_radius;
    float angle_PI;
};

void IR::begin()
{
    for (int i = 0; i < IR_NUM; i++)
    {
        unit_angle[i] = IR_IN[i];
        unit_cos[i] = cos(unit_angle[i]);
        unit_sin[i] = sin(unit_angle[i]);
    }
}

int IR::read(int channel)
{
    int controlPin[] = {s0, s1, s2, s3};
    for (int i = 0; i < 4; i++)
    {
        digitalWrite(controlPin[i], muxChannel[channel][i]);
    }
    int val = analogRead(SIG_pin);
    return val;
}

void IR::print()
{
    for (int i = 0; i < IR_NUM; i++)
    {
        IR_Cur[i] = IR_CUR_MAX - read(i);
        Serial.print(i);
        Serial.print(": ");
        Serial.println(IR_Cur[i]);
    }
}

float IR::get_radius()
{
    vector_XY = {0, 0};
    for (int i = 0; i < IR_NUM; i++)
    {
        vector_XY.x += IR_Cur[i] * unit_cos[i];
        vector_XY.y += IR_Cur[i] * unit_sin[i];
    }

    vector_RT.radius = sqrt(pow(vector_XY.x, 2.0) + pow(vector_XY.y, 2.0));
    now_radius = ave.updateData(vector_RT.radius);
    return now_radius;
}

float IR::get_angle()
{
    vector_RT.angle = atan2(vector_XY.y, vector_XY.x);
    angle_PI = vector_RT.angle / PI;
    return angle_PI;
}

#endif
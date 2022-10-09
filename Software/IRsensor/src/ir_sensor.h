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
    void IR_get();
    void IRpin_read();
    void radius_read();
    void angle_read();
    void send();
    float angle_PI;

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
    int IR_Cur[IR_NUM];
    float IR_IN[IR_NUM] = {0, -PI / 8, -PI * 2 / 8, -PI * 3 / 8, -PI * 4 / 8, -PI * 5 / 8, -PI * 6 / 8, -PI * 7 / 8, PI, PI * 7 / 8, PI * 6 / 8, PI * 5 / 8, PI * 4 / 8, PI * 3 / 8, PI * 2 / 8, PI / 8}; //ピンの角度
    float IR_cor[IR_NUM] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    float unit_angle[IR_NUM];
    float unit_sin[IR_NUM];
    float unit_cos[IR_NUM];
    // float unit_cor[IR_NUM];
    float now_radius;
    int maxPinVal = 0;
    int maxPin = 0;
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

void IR::IR_get()
{
    int controlPin[] = {s0, s1, s2, s3};
    for (int i = 0; i < IR_NUM - 1; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            digitalWrite(controlPin[j], muxChannel[i][j]);
        }
        IR_Cur[i] = IR_CUR_MAX - analogRead(SIG_pin);
    }

    IR_Cur[15] = IR_CUR_MAX - analogRead(A0);

    vector_XY = {0, 0};
    for (int i = 0; i < IR_NUM; i++)
    {
        vector_XY.x += IR_Cur[i] * unit_cos[i];
        vector_XY.y += IR_Cur[i] * unit_sin[i];
    }
    vector_RT.radius = sqrt(pow(vector_XY.x, 2.0) + pow(vector_XY.y, 2.0));
    now_radius = ave.updateData(vector_RT.radius); //半径

    vector_RT.angle = atan2(vector_XY.y, vector_XY.x);
    angle_PI = vector_RT.angle / PI; //角度
}

void IR::IRpin_read()
{

    for (int i = 0; i < IR_NUM; i++)
    {
        Serial.print(i);
        Serial.print(": ");
        Serial.println(IR_Cur[i]);
    }
    Serial.println("ー－－－－");
    /*
    for (int i = maxPin - 2 + 16; i < maxPin - 2 + 16 + 5; i++)
    {
        i = i % 16;
        Serial.print(i);
        Serial.print(": ");
        Serial.println(IR_Cur[i]);
    }*/
}

void IR::radius_read()
{
    Serial.print("radius:");
    Serial.println(now_radius);
}

void IR::angle_read()
{
    Serial.print("angle:");
    Serial.print(angle_PI);
    Serial.println("π");
}

void IR::send()
{
    float _angle = angle_PI;
    _angle += 1;
    int angle = _angle * 100;
    Serial.print(angle_PI);
    Serial.print(", ");
    Serial.println(angle);
    //Serial1.write(255);
    Serial1.write(angle);
}
#endif
#include <Arduino.h>
// #include <moving_average.h>

#ifndef __IR_SENSOR__
#define __IR_SENSOR__

#define IR_NUM 16
#define IR_CUR_MAX 1023
#define IR_SENSOR_RADIUS 5.0
#define s0 2
#define s1 3
#define s2 4
#define s3 5
#define SIG_pin A1
// #define kLPF 0
#define kLPF 0.2
#define MOVE_AVE_NUM 100
//  MovingAverage ave(100);

class IR
{
public:
    void begin();
    void IR_get();
    void IRpin_read();
    void IRonepin_read(int pinnum);
    void radius_read();
    void angle_read();
    void send();
    float angle_PI;
    float now_radius;
    bool ball_flag = false;

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
    int IR_Cur[IR_NUM];
    int IR_Cur_LPF[IR_NUM];
    float IR_Cur_Length[IR_NUM];
    float IR_IN[IR_NUM] = {
        0, -PI / 8, -PI * 2 / 8, -PI * 3 / 8,
        -PI * 4 / 8, -PI * 5 / 8, -PI * 6 / 8, -PI * 7 / 8,
        PI, PI * 7 / 8, PI * 6 / 8, PI * 5 / 8,
        PI * 4 / 8, PI * 3 / 8, PI * 2 / 8, PI / 8}; // ピンの角度
    float IR_cor[IR_NUM] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    float unit_angle[IR_NUM];
    float unit_sin[IR_NUM];
    float unit_cos[IR_NUM];
    float pre_radius[MOVE_AVE_NUM] = {0};
    // float unit_cor[IR_NUM];
};

void IR::begin()
{
    for (int i = 0; i < IR_NUM; i++)
    {
        unit_angle[i] = IR_IN[i];
        unit_cos[i] = cos(unit_angle[i]);
        unit_sin[i] = sin(unit_angle[i]);
    }
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    pinMode(SIG_pin, INPUT);
}

void IR::IR_get()
{
    int controlPin[] = {s0, s1, s2, s3};
    for (int i = 0; i < IR_NUM; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            digitalWrite(controlPin[j], muxChannel[i][j]);
        }
        delayMicroseconds(1);
        IR_Cur[i] = IR_CUR_MAX - analogRead(SIG_pin);
        IR_Cur_LPF[i] = kLPF * IR_Cur[i] + (1 - kLPF) * IR_Cur_LPF[i];
    }
    // IR_Cur[2] = IR_CUR_MAX - analogRead(A0);
    // IR_Cur_LPF[2] = kLPF * IR_Cur[2] + (1 - kLPF) * IR_Cur_LPF[2];
    //  IR_Cur_LPF[2] = (IR_Cur_LPF[1] + IR_Cur_LPF[3]) / 2;
    int sum = 0;
    for (int i = 0; i < IR_NUM; i++)
    {
        sum += IR_Cur_LPF[i];
    }
    if (sum < 100)
    {
        ball_flag = false;
        return;
    }
    else
    {
        ball_flag = true;
        int maxVal = 0;
        int maxIndex = 0;
        for (int i = 0; i < IR_NUM; i++)
        {
            if (IR_Cur_LPF[i] > maxVal)
            {
                maxVal = IR_Cur_LPF[i];
                maxIndex = i;
            }
        }
        vector_XY = {0, 0};
        vector_RT.radius = 0;
        /*for (int i = 0; i < 5; i++)
        {
            int index = (maxIndex + i - 2) % IR_NUM;
            vector_XY.x += IR_Cur_LPF[index] * unit_cos[index];
            vector_XY.y += IR_Cur_LPF[index] * unit_sin[index];
            IR_Cur_Length[index] = -0.4408 * IR_Cur_LPF[index] + 372.54;
        }*/
        for (int i = 0; i < IR_NUM; i++)
        {
            vector_XY.x += IR_Cur_LPF[i] * unit_cos[i];
            vector_XY.y += IR_Cur_LPF[i] * unit_sin[i];
            IR_Cur_Length[i] = -0.4408 * IR_Cur_LPF[i] + 372.54;
        }
        vector_RT.angle = atan2(vector_XY.y, vector_XY.x);
        vector_RT.radius = -0.4408 * IR_Cur_LPF[maxIndex] + 372.54;
        for (int i = 0; i < MOVE_AVE_NUM - 1; i++)
        {
            pre_radius[i] = pre_radius[i + 1];
        }
        pre_radius[MOVE_AVE_NUM - 1] = vector_RT.radius;
        float ave_radius = 0;
        for (int i = 0; i < MOVE_AVE_NUM; i++)
        {
            ave_radius += pre_radius[i];
        }
        ave_radius /= MOVE_AVE_NUM;
        now_radius = ave_radius;
        angle_PI = vector_RT.angle / PI; // 角度
    }
}

void IR::IRpin_read()
{
    /*
    for (int i = 0; i < IR_NUM; i++) {
        Serial.print(i);
        Serial.print("\t");
    }
    Serial.println();
    */
    for (int i = 0; i < IR_NUM; i++)
    {
        Serial.print(IR_Cur[i]);
        Serial.print("\t");
    }
    Serial.println();
}

void IR::IRonepin_read(int pinnum) { Serial.println(IR_Cur[pinnum]); }

void IR::radius_read()
{
    // Serial.print("radius: ");
    Serial.print(vector_RT.radius);
    Serial.print(",");
    Serial.println(now_radius);
}

void IR::angle_read()
{
    Serial.print("angle: ");
    Serial.print(angle_PI);
    Serial.println("π");
}

void IR::send()
{
    int sign = 0;
    if (angle_PI < 0)
    {
        sign = 1;
        angle_PI *= -1;
    }
    int strech_deg = round(angle_PI * 255.0);
    int ir_dist1 = now_radius / 10;
    uint8_t ir_dist2 = (fmod(now_radius, 10)) * 10;
    Serial1.write(255);
    Serial1.write(sign);
    Serial1.write(strech_deg);
    Serial1.write(ir_dist1);
    Serial1.write(ir_dist2);
}
#endif
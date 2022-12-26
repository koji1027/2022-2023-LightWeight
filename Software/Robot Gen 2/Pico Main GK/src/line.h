#ifndef LINE_H
#define LINE_H
#include <Arduino.h>

#define SENSOR_NUM 32
#define THRESHOLD 400
#define LINE_SENSOR_RADIUS 53.0

class Line {
   public:
    void begin();
    void read();
    void print();  // デバッグ用
    bool entire_sensor_state = false;
    float line_vector[2] = {0.0};
    float line_theta = 0.0;
    float line_dist = 0.0;

   private:
    const int COM_PIN[2] = {A0, A1};
    const int SELECT_PIN[2][4] = {{D2, D3, D6, D7}, {D10, D11, D12, D13}};
    float SENSOR_THETA[SENSOR_NUM] = {0.0};
    float SENSOR_X[SENSOR_NUM] = {0.0};
    float SENSOR_Y[SENSOR_NUM] = {0.0};
    bool sensor_state[SENSOR_NUM] = {false};
    int sensor_value[SENSOR_NUM] = {0};
    int cluster_num = 0;
    int cluster[8][15] = {0};
    int cluster_len[8] = {0};
};

void Line::begin() {
    for (int i = 0; i < 2; i++) {
        pinMode(COM_PIN[i], INPUT);
        for (int j = 0; j < 4; j++) {
            pinMode(SELECT_PIN[i][j], OUTPUT);
        }
    }
    for (int i = 0; i < 8; i++) {
        SENSOR_THETA[i] = PI / 8.0 * i;
    }
    for (int i = 8; i < 16; i++) {
        SENSOR_THETA[i] = PI / 8.0 * i - PI * 2.0;
    }
    for (int i = 0; i < SENSOR_NUM; i++) {
        SENSOR_X[i] = sin(SENSOR_THETA[i]);
        SENSOR_Y[i] = cos(SENSOR_THETA[i]);
    }
}

void Line::read() {
    int _sensor_value[32] = {0};
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 4; j++) {
            digitalWrite(SELECT_PIN[0][j], (byte)i & (1 << j));
            digitalWrite(SELECT_PIN[1][j], (byte)i & (1 << j));
        }
        // delayMicroseconds(10);
        _sensor_value[i] = analogRead(COM_PIN[0]);
        _sensor_value[i + 16] = 0;
    }
    sensor_value[0] = _sensor_value[1];
    sensor_value[1] = _sensor_value[3];
    sensor_value[2] = _sensor_value[5];
    sensor_value[3] = _sensor_value[7];
    sensor_value[4] = _sensor_value[9];
    sensor_value[5] = _sensor_value[11];
    sensor_value[6] = _sensor_value[13];
    sensor_value[7] = _sensor_value[15];
    sensor_value[8] = _sensor_value[0];
    sensor_value[9] = _sensor_value[2];
    sensor_value[10] = _sensor_value[4];
    sensor_value[11] = _sensor_value[6];
    sensor_value[12] = _sensor_value[8];
    sensor_value[13] = _sensor_value[10];
    sensor_value[14] = _sensor_value[12];
    sensor_value[15] = _sensor_value[14];

    for (int i = 0; i < 16; i++) {
        sensor_state[i] = sensor_value[i] > THRESHOLD;
    }

    int white_sensor_num = 0;
    int white_sensor[16];
    for (int i = 0; i < 16; i++) {
        if (sensor_state[i]) {
            white_sensor[white_sensor_num] = i;
            white_sensor_num++;
        }
    }
    if (white_sensor_num == 0) {
        entire_sensor_state = false;
        line_vector[0] = 0.0;
        line_vector[1] = 0.0;
        line_theta = 0.0;
        line_dist = 0.0;
        return;
    } else if (white_sensor_num == 1) {
        entire_sensor_state = true;
        line_vector[0] = SENSOR_X[white_sensor[0]];
        line_vector[1] = SENSOR_Y[white_sensor[0]];
        line_theta = SENSOR_THETA[white_sensor[0]];
        line_dist = LINE_SENSOR_RADIUS;
        return;
    } else {
        entire_sensor_state = true;
        int diff[white_sensor_num];
        for (int i = 0; i < white_sensor_num - 1; i++) {
            diff[i] = white_sensor[i + 1] - white_sensor[i];
        }
        diff[white_sensor_num - 1] =
            white_sensor[0] + 16 - white_sensor[white_sensor_num - 1];
        int max_diff = 0;
        int max_diff_index = 0;
        for (int i = 0; i < white_sensor_num; i++) {
            if (diff[i] > max_diff) {
                max_diff = diff[i];
                max_diff_index = i;
            }
        }
        if (max_diff % 2) {
            line_theta = SENSOR_THETA[white_sensor[max_diff_index +
                                                   (max_diff - 1) / 2]] +
                         PI / 16.0;
        } else {
            line_theta =
                SENSOR_THETA[white_sensor[max_diff_index + max_diff / 2]];
        }
    }
}
void Line::print() {
    if (entire_sensor_state) {
        Serial.println(line_theta);
    }
}
#endif
#ifndef LINE_H
#define LINE_H
#include <Arduino.h>

#define SENSOR_NUM 32
#define THRESHOLD 300

class Line {
   public:
    void begin();
    void read();
    void print();  // デバッグ用

   private:
    const int COM_PIN[2] = {A0, A1};
    const int SELECT_PIN[2][4] = {{D2, D3, D6, D7}, {D10, D11, D12, D13}};
    float SENSOR_THETA[SENSOR_NUM] = {PI,
                                      0.0,
                                      -7 * PI / 8.0,
                                      PI / 8.0,
                                      -6 * PI / 8.0,
                                      2 * PI / 8.0,
                                      -5 * PI / 8.0,
                                      3 * PI / 8.0,
                                      -4 * PI / 8.0,
                                      4 * PI / 8.0,
                                      -3 * PI / 8.0,
                                      5 * PI / 8.0,
                                      -2 * PI / 8.0,
                                      6 * PI / 8.0,
                                      -PI / 8.0,
                                      7 * PI / 8.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0};
    float SENSOR_X[SENSOR_NUM] = {0.0};
    float SENSOR_Y[SENSOR_NUM] = {0.0};
    int sensor_value[SENSOR_NUM] = {0};
    bool sensor_state[SENSOR_NUM] = {false};
    float line_vector[2] = {0.0};
    float line_theta = 0.0;
};

void Line::begin() {
    for (int i = 0; i < 2; i++) {
        pinMode(COM_PIN[i], INPUT);
        for (int j = 0; j < 4; j++) {
            pinMode(SELECT_PIN[i][j], OUTPUT);
        }
    }
    for (int i = 0; i < SENSOR_NUM; i++) {
        SENSOR_X[i] = sin(SENSOR_THETA[i]);
        SENSOR_Y[i] = cos(SENSOR_THETA[i]);
    }
}

void Line::read() {
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 4; j++) {
            digitalWrite(SELECT_PIN[0][j], (byte)i & (1 << j));
            digitalWrite(SELECT_PIN[1][j], (byte)i & (1 << j));
        }
        delayMicroseconds(10);
        sensor_value[i] = analogRead(COM_PIN[0]);
        sensor_value[i + 16] = analogRead(COM_PIN[1]);
    }
    sensor_value[18] = 0;
    for (int i = 0; i < SENSOR_NUM; i++) {
        if (sensor_value[i] > THRESHOLD) {
            sensor_state[i] = true;
        } else {
            sensor_state[i] = false;
        }
    }
    float line_vector_x = 0.0;
    float line_vector_y = 0.0;
    for (int i = 0; i < 16; i++) {
        if (sensor_state[i]) {
            line_vector_x += SENSOR_X[i];
            line_vector_y += SENSOR_Y[i];
        }
    }
    line_vector[0] = line_vector_x;
    line_vector[1] = line_vector_y;
    line_theta = atan2(line_vector[0], line_vector[1]);
}

void Line::print() {
    // デバッグ用
    Serial.print("line_theta: ");
    Serial.print(line_theta);
    Serial.println();
}

#endif
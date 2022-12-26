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

    int cluster[8][2] = {{0, 0}};
    int cluster_num = 0;
    int now_c = -1;
    for (int i = 0; i < 16; i++) {
        if (sensor_value[i] > THRESHOLD) {
            if (i > 0) {
                if (sensor_value[i - 1] > THRESHOLD) {
                    cluster[now_c][0] += 1;
                } else {
                    now_c += 1;
                    cluster_num += 1;
                    cluster[now_c][0] = 1;
                    cluster[now_c][1] = i;
                }
            } else {
                if (sensor_value[15] > THRESHOLD) {
                    cluster[now_c][0] += 1;
                } else {
                    now_c += 1;
                    cluster_num += 1;
                    cluster[now_c][0] = 1;
                    cluster[now_c][1] = i;
                }
            }
        }
    }
    if (cluster_num == 0) {
        entire_sensor_state = false;
        return;
    } else {
        entire_sensor_state = true;
        float cluster_vector[cluster_num][2] = {{0.0}};  // x,y
        float cluster_theta[cluster_num] = {0.0};

        for (int i = 0; i < cluster_num; i++) {
            if (cluster[i][0] % 2 == 0) {
                int center_index = cluster[i][0] / 2 - 1;
                cluster_vector[i][0] =
                    sin(SENSOR_THETA[center_index] + PI / 16.0);
                cluster_vector[i][1] =
                    cos(SENSOR_THETA[center_index] + PI / 16.0);
            } else {
                int center_index = (cluster[i][0] - 1) / 2;
                cluster_vector[i][0] =
                    sin(SENSOR_THETA[center_index + cluster[i][1]]);
                cluster_vector[i][1] =
                    cos(SENSOR_THETA[center_index + cluster[i][1]]);
            }
            cluster_theta[i] =
                atan2(cluster_vector[i][0], cluster_vector[i][1]);
        }
        if (cluster_num == 1) {
            line_vector[0] = cluster_vector[0][0];
            line_vector[1] = cluster_vector[0][1];
            line_theta = cluster_theta[0];
            line_dist = LINE_SENSOR_RADIUS;
        } else if (cluster_num == 2) {
            line_vector[0] = cluster_vector[0][0] + cluster_vector[1][0];
            line_vector[1] = cluster_vector[0][1] + cluster_vector[1][1];
            line_theta = atan2(line_vector[0], line_vector[1]);
            line_dist =
                abs(LINE_SENSOR_RADIUS * cos(PI - abs(cluster_theta[0] / 2.0) -
                                             abs(cluster_theta[1] / 2.0)));
        } else {
            line_vector[0] = 0.0;
            line_vector[1] = 0.0;
            for (int i = 0; i < cluster_num; i++) {
                line_vector[0] += cluster_vector[i][0];
                line_vector[1] += cluster_vector[i][1];
            }
            line_theta = atan2(line_vector[0], line_vector[1]);
            line_dist = LINE_SENSOR_RADIUS;
        }
        Serial.print("cluster ");
        for (int i = 0; i < cluster_num; i++) {
            Serial.print(i);
            Serial.print(": ");
            Serial.print(cluster[i][0]);
            Serial.print("個");
            Serial.print("  ");
            Serial.print(cluster[i][1]);
            Serial.print("番から");
            Serial.print("\t");
        }
        Serial.println();
    }
    /*sensor_value[18] = 0;
    for (int i = 0; i < SENSOR_NUM; i++) {
        if (sensor_value[i] > THRESHOLD) {
            sensor_state[i] = true;
        } else {
            sensor_state[i] = false;
        }
    }
    entire_sensor_state = false;
    for (int i = 0; i < SENSOR_NUM; i++) {
        if (sensor_state[i] == true) {
            entire_sensor_state = true;
        }
    }
    if (entire_sensor_state) {
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
        float line_sensor_theta = 0.0;
        for (int i = 0; i < 16; i++) {
            if (sensor_state[i]) {
                line_sensor_theta = SENSOR_THETA[i];
                break;
            }
        }
        line_length = LINE_RADIUS * cos(line_sensor_theta - line_theta);
    }*/
}

void Line::print() {}

#endif
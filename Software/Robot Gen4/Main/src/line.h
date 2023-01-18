#ifndef LINE_H
#define LINE_H
#include <Arduino.h>

#define SENSOR_NUM 32
#define LINE_SENSOR_RADIUS 53.0

class Line {
   public:
    void begin();
    void read();
    void print();
    void onepin_print(int i);  // デバッグ用
    void set_threshold();
    bool entire_sensor_state = false;
    float line_vector[2] = {0.0};
    float line_theta = 0.0;
    float line_dist = 0.0;

   private:
    const int COM_PIN[2] = {A1, A0};
    const int SELECT_PIN[2][4] = {{D8, D9, D10, D11}, {D2, D3, D6, D7}};
    int THRESHOLD[32] = {0};
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
    for (int i = 0; i < SENSOR_NUM; i++) {
        SENSOR_THETA[i] = (float)i * PI / 16.0;
    }
    for (int i = 0; i < SENSOR_NUM; i++) {
        SENSOR_X[i] = sin(SENSOR_THETA[i]);
        SENSOR_Y[i] = cos(SENSOR_THETA[i]);
    }
    for (int i = 0; i < SENSOR_NUM; i++) {
        THRESHOLD[i] = 900;
    }
}

void Line::read() {
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 4; j++) {
            digitalWrite(SELECT_PIN[0][j], (byte)i & (1 << j));
            digitalWrite(SELECT_PIN[1][j], (byte)i & (1 << j));
        }
        sensor_value[i + 16] = analogRead(COM_PIN[0]);
        sensor_value[i] = analogRead(COM_PIN[1]);
    }

    int posILW[SENSOR_NUM] = {0};  // 白線上にあるセンサの番号を格納
    int numILW = 0;                // 白線上にあるセンサの数
    for (int i = 0; i < SENSOR_NUM; i++) {
        if (sensor_value[i] > THRESHOLD[i]) {
            sensor_state[i] = true;
            posILW[numILW] = i;
            numILW++;
        } else {
            sensor_state[i] = false;
        }
    }
    if (numILW == 0) {
        entire_sensor_state = false;
        return;
    } else if (numILW == 1) {
        line_theta = SENSOR_THETA[posILW[0]];
        entire_sensor_state = true;
        return;
    } else {
        entire_sensor_state = true;
        int intvLine[numILW] = {0};  // 白線上にあるセンサの間隔
        for (int i = 0; i < numILW; i++) {
            intvLine[i] = posILW[i + 1] - posILW[i];
        }
        intvLine[numILW - 1] = posILW[0] + 32 - posILW[numILW - 1];

        int maxIntvL = -1;    // 最大の間隔
        int posMaxIntvL = 0;  // 最大の間隔の位置
        for (int i = 0; i < numILW; i++) {
            if (intvLine[i] > maxIntvL) {
                maxIntvL = intvLine[i];
                posMaxIntvL = i;
            }
        }

        if (maxIntvL % 2 == 1) {
            line_theta =
                SENSOR_THETA[posILW[posMaxIntvL] + (maxIntvL + 1) / 2] -
                PI / 32.0;
        } else {
            line_theta = SENSOR_THETA[posILW[posMaxIntvL] + maxIntvL / 2];
        }
        line_theta += PI;
        line_theta = fmod(line_theta, PI * 2.0);
        if (line_theta > PI) {
            line_theta -= PI * 2.0;
        }
    }
}
void Line::print() {
    for (int i = 0; i < 16; i++) {
        Serial.print(sensor_value[i]);
        Serial.print("\t");
    }
    Serial.println();
    delay(100);
}

void Line::set_threshold() {
    delay(1000);
    Serial.println(
        "しきい値設定を開始します。ロボットをコート上で動かしてください。");
    Serial.println("白線の上を通り過ぎるようにしてください。");
    Serial.println("3");
    delay(1000);
    Serial.println("2");
    delay(1000);
    Serial.println("1");
    delay(1000);
    Serial.println("Start");
    int maxValue[16] = {0};
    int minValue[16] = {1023};
    unsigned long long start_time = millis();
    while (millis() - start_time < 5000) {
        for (int i = 0; i < 16; i++) {
            for (int j = 0; j < 4; j++) {
                digitalWrite(SELECT_PIN[0][j], (byte)i & (1 << j));
            }
            int value = analogRead(COM_PIN[0]);
            if (value > maxValue[i]) {
                maxValue[i] = value;
            }
            if (value < minValue[i]) {
                minValue[i] = value;
            }
        }
    }
    for (int i = 0; i < 16; i++) {
        THRESHOLD[i] = (float)(maxValue[i] + minValue[i]) / 2.0;
    }
    THRESHOLD[15] = 400;
    Serial.println("自動しきい値設定完了！");
    for (int i = 0; i < 16; i++) {
        Serial.print(THRESHOLD[i]);
        Serial.print("\t");
    }
    Serial.println();
}
#endif
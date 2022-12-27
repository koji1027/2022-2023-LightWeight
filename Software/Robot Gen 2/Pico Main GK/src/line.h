#ifndef LINE_H
#define LINE_H
#include <Arduino.h>

#define SENSOR_NUM 32
#define LINE_SENSOR_RADIUS 53.0

class Line {
   public:
    void begin();
    void read();
    void print();  // デバッグ用
    void set_threshold();
    bool entire_sensor_state = false;
    float line_vector[2] = {0.0};
    float line_theta = 0.0;
    float line_dist = 0.0;

   private:
    const int COM_PIN[2] = {A0, A1};
    const int SELECT_PIN[2][4] = {{D2, D3, D6, D7}, {D10, D11, D12, D13}};
    int THRESHOLD[32] = {400};
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
    for (int i = 0; i < 16; i++) {
        THRESHOLD[i] = 400;
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

    int posILW[16] = {0};  // ライン上にあるセンサの番号を格納
    int numILW = 0;        // ライン上にあるセンサの数
    for (int i = 0; i < 16; i++) {
        if (sensor_value[i] > THRESHOLD[i]) {
            posILW[numILW] = i;
            numILW++;
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
        int intvLine[numILW] = {0};  // ライン上にあるセンサの間隔
        for (int i = 0; i < numILW; i++) {
            intvLine[i] = posILW[i + 1] - posILW[i];
        }
        intvLine[numILW - 1] = posILW[0] + 16 - posILW[numILW - 1];

        int minIntvL = 100;   // 最小の間隔
        int posMinIntvL = 0;  // 最小の間隔の位置
        for (int i = 0; i < numILW; i++) {
            if (intvLine[i] < minIntvL) {
                minIntvL = intvLine[i];
                posMinIntvL = i;
            }
        }

        if (minIntvL % 2 == 1) {
            line_theta =
                SENSOR_THETA[posILW[posMinIntvL] + (minIntvL + 1) / 2] -
                PI / 16.0;
        } else {
            line_theta = SENSOR_THETA[posILW[posMinIntvL] + minIntvL / 2];
        }
        line_theta = fmod(line_theta, PI * 2.0);
        if (line_theta > PI) {
            line_theta -= PI * 2.0;
        }
    }
}
void Line::print() {}

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
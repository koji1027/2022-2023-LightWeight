#include "ir.h"

// 赤外線センサーの初期化
void IR::begin(void) {
    pinMode(COM_PIN, INPUT);
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(MUX_PIN[i], OUTPUT);
    }
    for (uint8_t i = 0; i < IR_NUM; i++) {  // IRセンサの位置ベクトルを計算
        IR_SENSOR_VECTOR[i][0] = cos(-i * 2 * PI / IR_NUM);
        IR_SENSOR_VECTOR[i][1] = sin(-i * 2 * PI / IR_NUM);
    }
    analogReadResolution(ANALOG_READ_BIT);  // analogReadを12bitに設定
}

// 赤外線センサーの値からボールの角度と距離を計算
void IR::cal(void) {
    for (uint8_t i = 0; i < IR_NUM; i++) {  // 16個のセンサーの値を順番に読み取る
        // 4個のマルチプレクサの選択ピンを順番に選択
        for (uint8_t j = 0; j < 4; j++) {
            digitalWrite(MUX_PIN[j], (i >> j) & 0x01);
        }
        ir_val[i] = IR_VAL_MAX - analogRead(COM_PIN);  // IRセンサの値を反転
        // LPFをかける
        ir_val_LPF[i] = VALUE_LPF * ir_val[i] + (1 - VALUE_LPF) * ir_val_LPF[i];
    }

    uint16_t sum = 0;  // 16個のセンサーの値の合計
    for (uint8_t i = 0; i < IR_NUM; i++) {
        sum += ir_val_LPF[i];
    }
    // 16個のセンサーの値の合計が一定値以下のときボールがないと判断
    if (sum < 500) {
        ball_flag = false;
        return;
    } else {
        ball_flag = true;
    }

    double ir_vector[2] = {0, 0};  // ボールの位置ベクトル
    uint8_t max_index = 0;         // 最大のセンサーの値のインデックス
    for (uint8_t i = 0; i < IR_NUM; i++) {
        if (ir_val_LPF[i] > ir_val_LPF[max_index]) {
            max_index = i;
        }
    }
    for (uint8_t i = 0; i < 5; i++) {
        // 最大のセンサーの周囲5個のセンサーの値を使う
        uint8_t index = (max_index + i - 2 + 32) % IR_NUM;
        ir_vector[0] += ir_val_LPF[index] * IR_SENSOR_VECTOR[index][0];
        ir_vector[1] += ir_val_LPF[index] * IR_SENSOR_VECTOR[index][1];
    }
    ir_angle = atan2(ir_vector[1], ir_vector[0]);  // ボールの角度を計算
    // LPFをかける
    ir_angle_LPF = ir_angle_LPF * ANGLE_LPF + ir_angle * (1 - ANGLE_LPF);

    ir_angle = (ir_angle + ir_pre_angle) / 2.0;  // 移動平均
    ir_pre_angle = ir_angle;                     // 前回の角度を保存

    // ボールの距離を計算
    // 赤外線センサーの最大値を元に距離を計算．計算式は，Excelで実験したデータを元に分析した結果
    ir_dist = ((ir_val_LPF[max_index] * (-0.10471) + 346.74) + ir_dist) / 2.0;
}
#include "ir.h"

void IR::begin(void)
{
        pinMode(COM_PIN, INPUT);
        for (uint8_t i = 0; i < 4; i++)
        {
                pinMode(MUX_PIN[i], OUTPUT);
        }
        for (uint8_t i = 0; i < IR_NUM; i++)
        { // IRセンサの位置ベクトルを計算
                IR_SENSOR_VECTOR[i][0] = cos(-i * 2 * PI / IR_NUM);
                IR_SENSOR_VECTOR[i][1] = sin(-i * 2 * PI / IR_NUM);
        }
        analogReadResolution(ANALOG_READ_BIT); // analogReadを12bitに設定
}

void IR::cal(void)
{
        for (uint8_t i = 0; i < IR_NUM; i++)
        {
                for (uint8_t j = 0; j < 4; j++)
                {
                        digitalWrite(MUX_PIN[j], (i >> j) & 0x01);
                }
                ir_val[i] = IR_VAL_MAX - analogRead(COM_PIN);
                ir_val_lpf[i] = KLPF * ir_val[i] + (1 - KLPF) * ir_val_lpf[i]; // LPF
        }
        uint16_t sum = 0;
        for (uint8_t i = 0; i < IR_NUM; i++)
        {
                sum += ir_val_lpf[i];
        }
        if (sum < 500)
        {
                ball_flag = false; // ボールがないときは処理をスキップ
                return;
        }
        else
        {
                ball_flag = true;
        }
        double ir_vector[2] = {0, 0};
        uint8_t max_index = 0;
        for (uint8_t i = 0; i < IR_NUM; i++)
        {
                if (ir_val_lpf[i] > ir_val_lpf[max_index])
                {
                        max_index = i;
                }
        }
        for (uint8_t i = 0; i < 5; i++)
        {
                uint8_t index = (max_index + i - 2 + 32) % IR_NUM;
                ir_vector[0] += ir_val_lpf[index] * IR_SENSOR_VECTOR[index][0];
                ir_vector[1] += ir_val_lpf[index] * IR_SENSOR_VECTOR[index][1];
        }
        ir_angle = atan2(ir_vector[1], ir_vector[0]); // -PI ~ PI
        ir_angle = (ir_angle + ir_pre_angle) / 2.0;   // 移動平均
        ir_pre_angle = ir_angle;
        // ir_dist = sqrt(ir_vector[0] * ir_vector[0] + ir_vector[1] * ir_vector[1]); // ベクトルの大きさ √(x^2+y^2)
        ir_dist = ((ir_val_lpf[max_index] * (-0.10471) + 346.74) + ir_dist) / 2.0; // 移動平均
        /*for (uint8_t i = 0; i < IR_NUM; i++)
        {
                Serial.print(ir_val_lpf[i]);
                Serial.print("\t");
        }
        Serial.println();*/
}
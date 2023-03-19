#include "line.h"

void Line::begin()
{
        for (uint8_t i = 0; i < 2; i++)
        {
                pinMode(COM_PIN[i], INPUT);
                for (uint8_t j = 0; j < 4; j++)
                {
                        pinMode(SELECT_PIN[i][j], OUTPUT);
                }
        }
        for (uint8_t i = 0; i < SENSOR_NUM; i++)
        {
                SENSOR_THETA[i] = (float)i * PI / 16.0;
        }
        for (uint8_t i = 0; i < SENSOR_NUM; i++)
        {
                SENSOR_VECTOR[i][0] = sin(SENSOR_THETA[i]);
                SENSOR_VECTOR[i][1] = cos(SENSOR_THETA[i]);
        }
        for (uint8_t i = 0; i < SENSOR_NUM; i++)
        {
                threshold[i] = DEFAULT_THRESHOLD;
        }
}

void Line::read()
{
        pre_line_theta = line_theta;
        line_theta = 0.0;
        for (int i = 0; i < 16; i++)
        {
                for (int j = 0; j < 4; j++)
                {
                        digitalWrite(SELECT_PIN[0][j], (byte)i & (1 << j));
                        digitalWrite(SELECT_PIN[1][j], (byte)i & (1 << j));
                }
                sensor_value[i + 16] = analogRead(COM_PIN[0]);
                sensor_value[i] = analogRead(COM_PIN[1]);
        }

        int posILW[SENSOR_NUM] = {0}; // 白線上にあるセンサの番号を格納
        int numILW = 0;               // 白線上にあるセンサの数
        for (int i = 0; i < SENSOR_NUM; i++)
        {
                if (sensor_value[i] > threshold[i])
                {
                        sensor_state[i] = true;
                        posILW[numILW] = i;
                        numILW++;
                }
                else
                {
                        sensor_state[i] = false;
                }
        }
        sensor_value[12] = 0;
        sensor_state[12] = 0;
        cluster_num = -1; // (実際の数) -１
        int cluster[SENSOR_NUM] = {0};
        int cluster_size[SENSOR_NUM] = {0};
        for (int i = 0; i < SENSOR_NUM; i++)
        {
                if (sensor_state[i])
                {
                        if (cluster_num == -1)
                        {
                                cluster_num = 0;
                                cluster[cluster_num] = i;
                                cluster_size[cluster_num] = 1;
                        }
                        else if (cluster[cluster_num] + cluster_size[cluster_num] == i)
                        {
                                cluster_size[cluster_num]++;
                        }
                        else
                        {
                                cluster_num++;
                                cluster[cluster_num] = i;
                                cluster_size[cluster_num] = 1;
                        }
                }
        }
        if (cluster_num == -1)
        {
                on_line = false;
                return;
        }
        else
        {
                on_line = true;
                if (cluster[0] == 0 &&
                    cluster[cluster_num] + cluster_size[cluster_num] == SENSOR_NUM)
                {
                        cluster[0] = cluster[cluster_num];
                        cluster_size[0] += cluster_size[cluster_num];
                        cluster_num--;
                }
                float cluster_theta[cluster_num + 1] = {0.0};
                for (int i = 0; i <= cluster_num; i++)
                {
                        if (cluster_size[i] % 2)
                        {
                                cluster_theta[i] =
                                    SENSOR_THETA[cluster[i] + (cluster_size[i] - 1) / 2];
                        }
                        else
                        {
                                cluster_theta[i] =
                                    SENSOR_THETA[cluster[i] + cluster_size[i] / 2] - PI / 32.0;
                        }
                }
                float sum_vector[2] = {0.0, 0.0};
                for (int i = 0; i <= cluster_num; i++)
                {
                        sum_vector[0] += sin(cluster_theta[i]);
                        sum_vector[1] += cos(cluster_theta[i]);
                }
                line_theta = atan2(sum_vector[0], sum_vector[1]);
        }
}

void Line::set_threshold()
{
        int maxValue[32];
        int minValue[32];
        for (int i = 0; i < 32; i++)
        {
                maxValue[i] = 0;
                minValue[i] = 1023;
        }
        bool loop_flag = true;
        while (loop_flag)
        {
                for (int i = 0; i < 16; i++)
                {
                        for (int j = 0; j < 4; j++)
                        {
                                digitalWrite(SELECT_PIN[0][j], (byte)i & (1 << j));
                                digitalWrite(SELECT_PIN[1][j], (byte)i & (1 << j));
                        }
                        int value0 = analogRead(COM_PIN[0]);
                        int value1 = analogRead(COM_PIN[1]);
                        if (value0 > maxValue[i])
                        {
                                maxValue[i] = value0;
                        }
                        if (value0 < minValue[i])
                        {
                                minValue[i] = value0;
                        }
                        if (value1 > maxValue[i + 16])
                        {
                                maxValue[i + 16] = value1;
                        }
                        if (value1 < minValue[i + 16])
                        {
                                minValue[i + 16] = value1;
                        }
                }
                if (digitalRead(D19) == LOW)
                {
                        loop_flag = false;
                }
        }
        for (int i = 0; i < 32; i++)
        {
                threshold[i] = (float)(maxValue[i] + minValue[i]) / 2.0;
        }
        Serial.println("自動しきい値設定完了！");
        for (int i = 0; i < 32; i++)
        {
                Serial.print(threshold[i]);
                Serial.print("\t");
        }
        Serial.println();
}

void Line::debug()
{
        for (int i = 0; i < 16; i++)
        {
                Serial.print(sensor_value[i]);
                Serial.print("\t");
        }
        Serial.println();
        for (int i = 16; i < 32; i++)
        {
                Serial.print(sensor_value[i]);
                Serial.print("\t");
        }
        Serial.println();
        Serial.println();
}
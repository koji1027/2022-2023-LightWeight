#ifndef LINE_H
#define LINE_H
#include <Arduino.h>

#define SENSOR_NUM 32
#define LINE_SENSOR_RADIUS 53.0
#define DEFAULT_THRESHOLD 700

class Line
{
public:
        void begin();
        void read();
        void debug();
        void set_threshold();
        int on_line = false;
        bool floor_flag = false;
        double line_theta = 0.0;
        double pre_line_theta = 0.0;
        int cluster_num = -1; // 実際の数-１
        int line_state_flag = 0;

private:
        const int COM_PIN[2] = {A1, A0};
        const int SELECT_PIN[2][4] = {{D8, D9, D10, D11}, {D2, D3, D6, D7}};
        double SENSOR_THETA[SENSOR_NUM] = {0.0};
        double SENSOR_VECTOR[SENSOR_NUM][2] = {0.0};
        uint16_t threshold[SENSOR_NUM] = {577, 584, 590, 586, 627, 604, 601, 614, 594, 605, 610, 627, 594, 615, 560, 508, 146, 498, 558, 578, 605, 592, 590, 608, 595, 619, 599, 586, 601, 577, 571, 576};
        uint16_t sensor_value[SENSOR_NUM] = {0};
        bool sensor_state[SENSOR_NUM] = {false};
};

#endif

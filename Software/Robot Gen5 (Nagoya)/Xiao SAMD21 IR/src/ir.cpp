#include "ir.h"

void IR::begin(void)
{
        pinMode(COM_PIN, INPUT);
        for (uint8_t i = 0; i < 4; i++)
        {
                pinMode(MUX_PIN[i], OUTPUT);
        }
        for (uint8_t i = 0; i < IR_NUM; i++)
        {
                IR_SENSOR_VECTOR[i][0] = cos(i * 2 * PI / IR_NUM);
                IR_SENSOR_VECTOR[i][1] = sin(i * 2 * PI / IR_NUM);
        }
}
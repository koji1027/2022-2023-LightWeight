#include <Arduino.h>

#define IR_NUM 16
#define ANALOG_READ_BIT 12
#define IR_VAL_MAX (1 << ANALOG_READ_BIT)
#define KLPF 0.2

class IR
{
public:
        void begin(void);
        void cal(void);
        double ir_angle = 0.0;
        double ir_dist;
        bool ball_flag = false;

private:
        const uint8_t COM_PIN = A1;
        const uint8_t MUX_PIN[4] = {2, 3, 4, 5};
        double IR_SENSOR_VECTOR[IR_NUM][2];
        double ir_val[IR_NUM];     // IRセンサの値（反転済みの値）
        double ir_val_lpf[IR_NUM]; // LPF後のIRセンサの値
        double ir_pre_angle = 0.0;
};
#include <Arduino.h>

#define IR_NUM 16                          // IRセンサの数
#define ANALOG_READ_BIT 12                 // アナログ入力のビット数
#define IR_VAL_MAX (1 << ANALOG_READ_BIT)  // IRセンサの最大値
#define VALUE_LPF 0.2                      // IRセンサの値のLPFの係数
#define ANGLE_LPF 0.3                      // IRセンサの角度のLPFの係数

class IR {
   public:
    void begin(void);           // IRセンサの初期化
    void cal(void);             // IRボールの位置を計算
    double ir_angle = 0.0;      // IRボールの角度
    double ir_angle_LPF = 0.0;  // LPF後のIRボールの角度
    double ir_dist;             // IRボールの距離
    bool ball_flag = false;     // IRボールが存在するかどうか
    double ir_val_LPF[IR_NUM];  // LPF後のIRセンサの値

   private:
    const uint8_t COM_PIN = A1;               // アナログマルチプレクサのよみとりピン
    const uint8_t MUX_PIN[4] = {2, 3, 4, 5};  // アナログマルチプレクサの選択ピン
    double IR_SENSOR_VECTOR[IR_NUM][2];       // IRセンサの位置ベクトル
    double ir_val[IR_NUM];                    // IRセンサの値
    double ir_pre_angle = 0.0;                // 前回のIRボールの角度
};
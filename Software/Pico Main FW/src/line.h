#ifndef LINE_H
#define LINE_H
#include <Arduino.h>

#define SENSOR_NUM 32            // センサの数
#define LINE_SENSOR_RADIUS 53.0  // センサの半径(mm)
#define DEFAULT_THRESHOLD 700    // デフォルトのしきい値

class Line {
   public:
    void begin();                                // ラインセンサの初期化
    void read();                                 // ラインセンサの値の読み取り
    void debug();                                // デバッグ用
    void set_threshold();                        // しきい値の設定
    int on_line = false;                         // ライン上にあるかどうか
    bool floor_flag = false;                     // 床に機体が接地しているかどうか
    double line_theta = 0.0;                     // ラインの角度
    double pre_line_theta = 0.0;                 // 1ループ前のラインの角度
    int cluster_num = -1;                        // 実際の数-１(自分でもよくわからない)
    int line_state_flag = 0;                     // ラインの状態
    uint16_t ave_threshold = DEFAULT_THRESHOLD;  // しきい値の平均値

   private:
    const int COM_PIN[2] = {A1, A0};                                      // マルチプレクサのCOMピン
    const int SELECT_PIN[2][4] = {{D8, D9, D10, D11}, {D2, D3, D6, D7}};  // マルチプレクサのセレクトピン
    double SENSOR_THETA[SENSOR_NUM] = {0.0};                              // センサの角度
    double SENSOR_VECTOR[SENSOR_NUM][2] = {0.0};                          // センサのベクトル
    uint16_t threshold[SENSOR_NUM] = {0};                                 // しきい値
    uint16_t sensor_value[SENSOR_NUM] = {0};                              // センサの値
    bool sensor_state[SENSOR_NUM] = {false};                              // センサの状態
};

#endif

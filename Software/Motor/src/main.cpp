#include <Arduino.h>
#include <motor_control.h>

#define TRACK_SPEED 0
#define LINE_SPEED 0
#define WRAPAROUND_SPEED 0

motor Motor;

void ir_get();
void gyro_get();
void line_get();

float gyro_rad = 0.0;
float ir_rad = 0.0;
float ir_dist = 0.0;
bool line_whole_flag = false;
float line_rad = 0.0;
float line_x[32] = {0.0};
float line_y[32] = {0.0};

const float LINE_ANGLE[32] = {
    1.0 * PI / 16.0,   2.0 * PI / 16.0,   3.0 * PI / 16.0,   4.0 * PI / 16.0,
    5.0 * PI / 16.0,   6.0 * PI / 16.0,   7.0 * PI / 16.0,   8.0 * PI / 16.0,
    9.0 * PI / 16.0,   10.0 * PI / 16.0,  11.0 * PI / 16.0,  12.0 * PI / 16.0,
    13.0 * PI / 16.0,  14.0 * PI / 16.0,  15.0 * PI / 16.0,  16.0 * PI / 16.0,
    -15.0 * PI / 16.0, -14.0 * PI / 16.0, -13.0 * PI / 16.0, -12.0 * PI / 16.0,
    -11.0 * PI / 16.0, -10.0 * PI / 16.0, -9.0 * PI / 16.0,  -8.0 * PI / 16.0,
    -7.0 * PI / 16.0,  -6.0 * PI / 16.0,  -5.0 * PI / 16.0,  -4.0 * PI / 16.0,
    -3.0 * PI / 16.0,  -2.0 * PI / 16.0,  -1.0 * PI / 16.0,  0.0 * PI / 16.0,
};

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
    Serial4.begin(115200);
    Serial5.begin(115200);
    Motor.init();
    for (int i = 0; i < 32; i++) {
        line_x[i] = sin(LINE_ANGLE[i]);
        line_y[i] = cos(LINE_ANGLE[i]);
    }
    delay(2000);
}

void loop() {
    // put your main code here, to run repeatedly:
    while (1) {
        gyro_get();
        ir_get();
        line_get();
        if (line_whole_flag) {
            if ((line_rad > -PI / 3.0 && line_rad < PI / 3.0) ||
                line_rad > 2.0 * PI / 3.0 || line_rad < -2.0 * PI / 3.0) {
                Motor.cal(line_rad + PI, LINE_SPEED, 0, 0);
            } else {
                Motor.cal(line_rad + PI, LINE_SPEED, 0, 0);
                delay(90);
            }
        } else {
            if (gyro_rad > PI / 2.0 || gyro_rad < -PI / 2.0) {
                Motor.cal(0, 0, 0, gyro_rad);
            } else {
                if (ir_rad >= 0 && ir_rad < radians(20)) {
                    Motor.cal(ir_rad * 1.5, WRAPAROUND_SPEED, 0, gyro_rad);
                } else if (ir_rad >= radians(20) && ir_rad < radians(45)) {
                    Motor.cal(radians(105), WRAPAROUND_SPEED, 0, gyro_rad);
                } else if (ir_rad >= radians(45) && ir_rad < radians(90)) {
                    Motor.cal(radians(120), WRAPAROUND_SPEED, 0, gyro_rad);
                } else if (ir_rad >= radians(90) && ir_rad < radians(150)) {
                    Motor.cal(radians(180), WRAPAROUND_SPEED, 0, gyro_rad);
                } else if (ir_rad >= radians(150) && ir_rad < radians(180)) {
                    Motor.cal(-radians(150), WRAPAROUND_SPEED, 0, gyro_rad);
                } else if (ir_rad >= -radians(180) && ir_rad < -radians(150)) {
                    Motor.cal(radians(150), WRAPAROUND_SPEED, 0, gyro_rad);
                } else if (ir_rad >= -radians(150) && ir_rad < -radians(90)) {
                    Motor.cal(radians(180), WRAPAROUND_SPEED, 0, gyro_rad);
                } else if (ir_rad >= -radians(90) && ir_rad < -radians(45)) {
                    Motor.cal(-radians(120), WRAPAROUND_SPEED, 0, gyro_rad);
                } else if (ir_rad >= -radians(45) && ir_rad < -radians(20)) {
                    Motor.cal(-radians(105), WRAPAROUND_SPEED, 0, gyro_rad);
                } else if (ir_rad >= -radians(20) && ir_rad < 0) {
                    Motor.cal(ir_rad * 1.5, WRAPAROUND_SPEED, 0, gyro_rad);
                } else {
                    Motor.cal(0, 0, 0, 0);
                }
            }
            delay(10);
        }
    }
}

void gyro_get() {
    Serial3.write(255);
    while (!Serial3.available()) {
    }
    int recv_data = Serial3.read();
    if (recv_data == 255) {
        while (!Serial3.available()) {
        }
        int sign = Serial3.read();
        int strech_rad = Serial3.read();
        int battery_voltage_flag = Serial3.read();
        gyro_rad = (float)strech_rad / 255.0 * PI;
        if (sign == 0) {
            gyro_rad *= -1.0;
        }
    }
}

void ir_get() {
    Serial2.write(255);
    while (!Serial2.available()) {
    }
    int recv_data = Serial2.read();
    if (recv_data == 255) {
        while (!Serial2.available()) {
        }
        int sign = Serial2.read();
        int _strech_ir_rad = Serial2.read();
        int ir_dist1 = Serial2.read();
        int ir_dist2 = Serial2.read();
        ir_rad = (float)_strech_ir_rad / 255.0 * PI;
        if (sign == 0) {
            ir_rad *= -1.0;
        }
        ir_rad += gyro_rad;
        ir_dist = (float)(ir_dist1 * 10 + (float)ir_dist2 / 10.0);
    }
}

void line_get() {
    bool line_flag[32] = {false};
    line_whole_flag = false;
    for (int i = 0; i < 32; i++) {
        line_flag[i] = 0;
    }
    int recv_data[4] = {0};
    Serial5.write(254);

    while (!Serial5.available()) {
    }
    if (Serial5.read() == 255) {
        for (int i = 0; i < 2; i++) {
            while (!Serial5.available()) {
            }
            recv_data[i] = Serial5.read();
        }
    }
    Serial4.write(254);
    while (!Serial4.available()) {
    }
    if (Serial4.read() == 255) {
        for (int i = 2; i < 4; i++) {
            while (!Serial4.available()) {
            }
            recv_data[i] = Serial4.read();
        }
    }
    /*while (!Serial5.available()) {
    }
    if (Serial5.read() == 255) {
        for (int i = 0; i < 2; i++) {
            while (!Serial5.available()) {
            }
            recv_data[i] = Serial5.read();
        }
    }
    Serial4.write(254);
    while (!Serial4.available()) {
    }
    if (Serial4.read() == 255) {
        for (int i = 2; i < 4; i++) {
            while (!Serial4.available()) {
            }
            recv_data[i] = Serial4.read();
        }
    }//*/

    for (int i = 0; i < 4; i++) {
        if (recv_data[i] > 127) {
            line_flag[(i + 1) * 8 - 1] = 1;
            recv_data[i] -= 128;
            line_whole_flag = true;
        }
        if (recv_data[i] > 63) {
            line_flag[(i + 1) * 8 - 2] = 1;
            recv_data[i] -= 64;
            line_whole_flag = true;
        }
        if (recv_data[i] > 31) {
            line_flag[(i + 1) * 8 - 3] = 1;
            recv_data[i] -= 32;
            line_whole_flag = true;
        }
        if (recv_data[i] > 15) {
            line_flag[(i + 1) * 8 - 4] = 1;
            recv_data[i] -= 16;
            line_whole_flag = true;
        }
        if (recv_data[i] > 7) {
            line_flag[(i + 1) * 8 - 5] = 1;
            recv_data[i] -= 8;
            line_whole_flag = true;
        }
        if (recv_data[i] > 3) {
            line_flag[(i + 1) * 8 - 6] = 1;
            recv_data[i] -= 4;
            line_whole_flag = true;
        }
        if (recv_data[i] > 1) {
            line_flag[(i + 1) * 8 - 7] = 1;
            recv_data[i] -= 2;
            line_whole_flag = true;
        }
        if (recv_data[i] > 0) {
            line_flag[(i + 1) * 8 - 8] = 1;
            recv_data[i] -= 1;
            line_whole_flag = true;
        }
    }

    /*for (int i = 0; i < 16; i++){
        Serial.print(line_flag[i]);
        Serial.print("\t");
    }
    Serial.print("\n");*/

    if (line_whole_flag) {
        float line_x_sum = 0;
        float line_y_sum = 0;
        for (int i = 0; i < 32; i++) {
            line_x_sum += line_x[i] * line_flag[i];
            line_y_sum += line_y[i] * line_flag[i];
        }
        line_rad = atan2(line_x_sum, line_y_sum);
        // Serial.println(line_deg);
    }
    // else{Serial.println("Yeah");}
    /*Serial4.write(252);
    for (int i = 0; i < 16; i++)
    {
        while (!Serial4.available())
        {
        }
        Serial.print(Serial4.read());
        Serial.print("\t");
    }
    Serial.print("\n");*/
    // Serial.println(line_deg);
}

#include <Arduino.h>

#include "MPU6050/gyro.h"
#include "led.h"
#include "line.h"

#define DIST_BALL -20.0
#define CIRC_BASE pow(0.75, 1.0 / 20.0)

SerialPIO motor(D17, D16, 32);
SerialPIO ir(D0, D1, 32);
Gyro gyro;
Line line;

float machine_angle = 0.0;
float ir_angle = 0.0;
float ir_radius = 0.0;
float move_angle = 0.0;
int ball_flag = 0;
int line_flag = 0;
bool start_flag = false;
int led_color[3] = {255, 255, 255};
int led_brightness = 50;

void setup() {
    Serial.begin(115200);
    gyro.begin();
    line.begin();
    set_led(led_color, led_brightness);
    init_led();
    delay(1000);
}

void loop() {
    gyro.getEuler();
    line.read();
}

void setup1() {
    motor.begin(115200);
    ir.begin(115200);
    delay(3000);
}

void loop1() {
    ir.write(255);
    if (ir.available() > 5) {
        int recv_data = ir.read();
        if (recv_data == 255) {
            byte data[4];
            data[0] = ir.read();
            data[1] = ir.read();
            data[2] = ir.read();
            data[3] = ir.read();
            float _ir_angle = (float)(data[0] + (data[1] << 8)) / 100.0 - PI;
            if (abs(_ir_angle) <= PI) {
                ir_angle = _ir_angle;
            }
            ir_radius = (float)data[2];
            ball_flag = data[3];
        }
    }
    float Circ_Kp = pow(CIRC_BASE, ir_radius);
    move_angle = ir_angle + ir_angle * Circ_Kp;
    Serial.println(ir_radius);
    float vx = sin(move_angle);
    float vy = cos(move_angle);
    
    if (line.entire_sensor_state){
        if((line.line_theta >= 0 && line.line_theta <= PI/4)||
           (line.line_theta > -PI/4 && line.line_theta <= 0)){
            vy = 0;
        }
        if(line.line_theta > PI/4 && line.line_theta <= 3*PI/4){
            vx = 0;
        }
        if((line.line_theta > 3*PI/4 && line.line_theta <= PI)||
           (line.line_theta >= -PI && line.line_theta <= -3*PI/4)){
            vy = 0;
        }
        if(line.line_theta > -3*PI/4 && line.line_theta >= -PI/4){
            vx = 0;
        }
    }
    
    //if (line.entire_sensor_state){vx=0; vy=0;}

    vx = (vx + 1.0) * 100.0;
    vy = (vy + 1.0) * 100.0;

    byte data[8];
    data[0] = vx;  // 送るもとの値は -1~1 だが、送るときは 0~200 にする
    data[1] = vy;
    data[2] = 100;
    data[3] = (byte)((gyro.angle + PI) * 100);
    data[4] = (byte)((int)((gyro.angle + PI) * 100) >> 8);
    data[5] = (byte)((machine_angle + PI) * 100);
    data[6] = (byte)((int)((machine_angle + PI) * 100) >> 8);
    data[7] = 0;

    motor.write(255);
    motor.write(data, 8);
    delay(10);
}
#include <Arduino.h>
#include <ir_sensor.h>
#include <moving_average.h>

#define ECHO 9
#define TRIG 10

unsigned long long pre_time = 0;
float distance = 0.0;
float pre_distance[10] = {0.0, 0.0, 0.0};
bool start_flag = false;

IR ir;

void setup() {
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    pinMode(SIG_pin, INPUT);
    pinMode(ECHO, INPUT);
    pinMode(TRIG, OUTPUT);

    digitalWrite(s0, LOW);
    digitalWrite(s1, LOW);
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);

    Serial.begin(115200);
    Serial1.begin(115200);

    ir.begin();
    while (!start_flag) {
        if (Serial1.available() > 0) {
            int recv_data = Serial1.read();
            if (recv_data == 255) {
                start_flag = true;
            }
        }
    }
}

void loop() {
    ir.IR_get();

    // ir.IRpin_read();
    //    ir.radius_read();
    //    ir.IRonepin_read(1);

    unsigned long long now = millis();
    if (now - pre_time > 50) {
        pre_time = now;
        digitalWrite(TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG, LOW);
        float duration = pulseIn(ECHO, HIGH);
        if (duration > 0) {
            distance = duration * (331.5 + 0.6 * 25) * 100 / 1000000;
            if (distance > 255) {
                distance = 70;
            }
        }
        for (int i = 0; i < 9; i++) {
            pre_distance[i] = pre_distance[i + 1];
        }
        pre_distance[9] = distance;
        int min_distance = 255;
        for (int i = 0; i < 10; i++) {
            if (pre_distance[i] < min_distance) {
                min_distance = pre_distance[i];
            }
        }
        distance = min_distance;
    }

    int a = (ir.angle_PI * PI + PI) * 100;
    byte data[3];
    data[0] = byte(a);
    data[1] = byte(a >> 8);
    data[2] = byte(distance);
    Serial1.write(255);
    Serial1.write(data, 3);
    // Serial.println("Boys Be Ambitious.");
}
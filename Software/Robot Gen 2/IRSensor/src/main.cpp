#include <Arduino.h>
#include <ir_sensor.h>
#include <moving_average.h>

IR ir;

void setup() {
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    pinMode(SIG_pin, INPUT);
    pinMode(A0, INPUT);

    digitalWrite(s0, LOW);
    digitalWrite(s1, LOW);
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);

    Serial.begin(115200);
    Serial1.begin(115200);

    ir.begin();
}

void loop() {
    ir.IR_get();
    ir.radius_read();
    // ir.angle_read();
    // ir.IRonepin_read(1);
    int recv_data = Serial1.read();
    if (recv_data == 255) {
        int a = (ir.angle_PI * PI + PI) * 100;
        int b = ir.now_radius;
        if (b < 0) {
            b = 0;
        }
        if (b > 255) {
            b = 255;
        }
        byte data[3];
        data[0] = byte(a);
        data[1] = byte(a >> 8);
        data[2] = byte(b);
        Serial1.write(255);
        Serial1.write(data, 3);
    }
    // delay(100);
    // Serial.println("Boys Be Ambitious.");
}
#include <Arduino.h>

#include "ir_sensor.h"
#include "sound.h"

IR ir;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial1.begin(115200);
    ir.begin();
    pinMode(SPEAKER_PIN, OUTPUT);
    while (!Serial1)
    {
        delay(10);
    }
}

void loop()
{
    // put your main code here, to run repeatedly:
    ir.IR_get();
    if (Serial1.available() > 0)
    {
        int recv_data = Serial1.read();
        if (recv_data == 255)
        {
            byte data[5];
            data[0] = 255;
            data[1] = (byte)((ir.angle_PI * PI + PI) * 100);
            data[2] = (byte)((int)((ir.angle_PI * PI + PI) * 100) >> 8);
            data[3] = (byte)ir.now_radius;
            data[4] = (byte)ir.ball_flag;
            Serial1.write(data, 5);
        }
    }
}

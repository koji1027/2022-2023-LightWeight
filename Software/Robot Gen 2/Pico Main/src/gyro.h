#include <Arduino.h>
#include <Wire.h>

#define GYRO_BUFFER_SIZE 1000

class Gyro {
   public:
    void begin();
    void calibration();
    void read();
    void calcAngle();
    float angle = 0.0;

   private:
    int gzRaw = 0;
    int gzRawOffset = 0;
    float gz = 0.0;
    unsigned long long pre_time = 0;
    float pre_gz = 0.0;
};

void Gyro::begin() {
    Wire.setClock(400000);
    Wire.begin();

    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    // ジャイロセンサー初期設定
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x18);
    Wire.endTransmission();
    // 加速度センサー初期設定
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    // LPF設定
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();
}

void Gyro::calibration() {
    Serial.println("Calibrating... ");
    Serial.println("Don't move the robot");

    unsigned long long gzRawSum = 0;
    for (int i = 0; i < 100; i++) {
        read();
        delay(10);
    }
    for (int i = 0; i < GYRO_BUFFER_SIZE; i++) {
        read();
        gzRawSum += gzRaw;
        delay(1);
    }
    gzRawOffset = round((float)gzRawSum / (float)GYRO_BUFFER_SIZE);
    Serial.println("Calibration done");
}

void Gyro::read() {
    Wire.beginTransmission(0x68);
    Wire.write(0x47);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 2);

    while (Wire.available() < 2)
        ;
    gzRaw = Wire.read() << 8 | Wire.read();
    gzRaw -= gzRawOffset;
    gzRaw = fmod(gzRaw, 65536);
    gz = gzRaw / 16.4 / 180.0 * PI;
    gz = gz > (2000.0 / 180.0 * PI) ? gz - (4000.0 / 180.0 * PI) : gz;
}

void Gyro::calcAngle() {
    unsigned long long now_time = micros();
    float dt = (now_time - pre_time) / 1000000.0;
    angle += (pre_gz + gz) / 2.0 * dt;
    if (angle < -PI) {
        angle += 2 * PI;
    } else if (angle > PI) {
        angle -= 2 * PI;
    }
    pre_time = now_time;
    pre_gz = gz;
}
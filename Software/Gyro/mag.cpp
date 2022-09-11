#include <Arduino.h>
#include <BMX055.h>
#include <math.h>

float radian;
float degree;

void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は11200bps
  Serial.begin(115200);
  // BMX055 初期化
  BMX055_Init();
  delay(300);
}

void loop(){
    while (true)
  {
    BMX055_Mag();
    radian = atan2(yMag,xMag);
    degree = radian * 180 / PI;
    Serial.print(degree);
    Serial.println("°");
  }
  delay(10);
}

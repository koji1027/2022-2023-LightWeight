#include <Arduino.h>
#include <BMX055.h>
#include <math.h>
#include <fstream>

int adjust_xMag = 25;
int adjust_yMag = 55;
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
    xMag += adjust_xMag;
    yMag += adjust_yMag;
    /*
    Serial.print(xMag);
    Serial.print(",");
    Serial.println(yMag);
    */
    radian = atan2(yMag,xMag);
    degree = radian * 180 / PI;
    Serial.print(degree);
    Serial.println("°");
    delay(10);
  }
}

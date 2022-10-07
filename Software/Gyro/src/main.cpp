#include <Arduino.h>
#include <BMX055.h>
float degree = 0.00;
float prexGyro = 0.00;
unsigned long long preMicros = 0;
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

void loop()
{
  // Serial.println("--------------------------------------");

  // BMX055 加速度の読み取り
  BMX055_Accl();
  Serial.print("Accl= ");
  Serial.print(xAccl);
  Serial.print(",");
  Serial.print(yAccl);
  Serial.print(",");
  Serial.print(zAccl);
  Serial.println("");

  // BMX055 ジャイロの読み取り
  BMX055_Gyro();
  Serial.print("Gyro= ");
  Serial.print(xGyro);
  Serial.print(",");
  Serial.print(yGyro);
  Serial.print(",");
  Serial.print(zGyro);
  Serial.println("");

  // BMX055 磁気の読み取り
  BMX055_Mag();
  Serial.print("Mag= ");
  Serial.print(xMag);
  Serial.print(",");
  Serial.print(yMag);
  Serial.print(",");
  Serial.print(zMag);
  Serial.println("");
  delay(100);
  /*while (true)
  {
    BMX055_Gyro();
    zGyro -= 0.01;
    unsigned long long time = micros();
    degree += (zGyro + prexGyro) * float(time - preMicros) / 2000000;
    preMicros = time;
    prexGyro = zGyro;
    Serial.print(degree);
    Serial.println("°");
  }
  delay(10);*/
}

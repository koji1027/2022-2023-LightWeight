#include <Arduino.h>
#include <motor_control.h>

#define CONST_ANG_VEL 0.5

void ir_get();
void gyro_get();

float ir_deg = 0;
float gyro_deg = 0;

motor_control Motor;

void setup()
{
  // put your setup code here, to run once:
  // Serial.begin(115200);
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Motor.begin();
  delay(2000);
}

void loop()
{
  // put your main code here, to run repeatedly:
  // ir_get();
  //gyro_get();
  // Serial.print("ir_deg: ");
  // Serial.print(ir_deg);
  // Serial.print("  gyro_deg: ");
  // Serial.println(gyro_deg);
  Motor.cal(0, 0, 150, 0, gyro_deg);
  delay(100);
  // Motor.cal(0, 0, CONST_ANG_VEL * gyro_deg, 255);
}

void ir_get()
{
  /*Serial2.write(255);
  while (!Serial2.available())
  {
  }
  int recv_data = Serial2.read();
  if (recv_data == 255)
  {
    recv_data = Serial2.read();
    gyro_deg = recv_data;
    /*recv_data /= 100.0;
    recv_data -= 1;
    ir_deg = recv_data * 180;
  }*/
  Serial2.write(255);
  while (!Serial2.available())
  {
  }
  int recv_data = Serial2.read();
  ir_deg = recv_data;
  ir_deg /= 100.0;
  ir_deg -= 1.0;
  ir_deg *= 180.0;
}

void gyro_get()
{
  Serial3.write(255);
  while (!Serial3.available())
  {
  }
  int recv_data = Serial3.read();
  if (recv_data == 255)
  {
    recv_data = Serial3.read();
    gyro_deg = recv_data;
    gyro_deg /= 100.0;
    gyro_deg -= 1.0;
    gyro_deg *= 180.0;
    if (gyro_deg > 180)
    {
      gyro_deg -= 360.0;
    }
  }
}
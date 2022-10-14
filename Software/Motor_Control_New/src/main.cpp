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
  ir_get();
  gyro_get();
  float ir_rad = ir_deg * PI / 180;
  Serial.print("gyro_deg : ");
  Serial.print(gyro_deg);
  Serial.print(" ir_deg : ");
  Serial.print(ir_rad);
  Serial.print(" x : ");
  Serial.print(cos(ir_rad));
  Serial.print(" y : ");
  Serial.println(sin(ir_rad));
  // Motor.cal(0, 1, 150, 0, 0);
  Motor.cal(sin(ir_rad), cos(ir_rad), 255, 0, gyro_deg);
  delay(10);
}

void ir_get()
{
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
    // Serial.println("OH,NO!");
  }
  int recv_data = Serial3.read();
  // Serial.println(recv_data);

  gyro_deg = recv_data;
  gyro_deg /= 100.0;
  gyro_deg -= 1.0;
  gyro_deg *= 180.0;

  // Serial.println(gyro_deg);

  /*
  Serial3.write(255);
  Serial.println(255);
  delay(10);*/
}
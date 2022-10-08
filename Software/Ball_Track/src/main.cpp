#include <Arduino.h>
#include <motorcontrol.h>
float ball_angle = 0;
float robo_angle = 0;
void setup()
{
  // put your setup code here, to run once:
  motor_init();
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  //motor_control(0, 0);
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial2.write(255);
  while (!Serial2.available())
  {
  }
  int data = Serial2.read();
  if (data == 255)
  {
    data = Serial2.read();
    data -= 100;
    ball_angle = float(data) / 100.0 * PI;
  }
  Serial.print("ball_angle:");
  Serial.print(ball_angle);
  Serial3.write(255);
  while (!Serial3.available())
  {
  }
  data = Serial3.read();
  if (data == 255)
  {
    data = Serial3.read();
    data -= 100;
    robo_angle = float(data) / 100.0 * PI;
  }
  Serial.print(" , robo_angle:");
  Serial.println(robo_angle);
  if (robo_angle > (15.0 / 180.0 * PI) || robo_angle < (-15.0 / 180.0 * PI))
  {
    spin(robo_angle);
  }
  else
  {
    motor_control(ball_angle, 150);
  }
  delay(50);
}
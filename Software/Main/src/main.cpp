#include <Arduino.h>
#include <motor_control.h>
#include <gyro.h>

#define SERIAL_BAUD_RATE 115200

Motor motor;
Axis axis;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(SERIAL_BAUD_RATE);
  Serial1.begin(SERIAL_BAUD_RATE);
  Serial2.begin(SERIAL_BAUD_RATE);
  Serial3.begin(SERIAL_BAUD_RATE);
  Serial4.begin(SERIAL_BAUD_RATE);
  Serial5.begin(SERIAL_BAUD_RATE);
  Serial6.begin(SERIAL_BAUD_RATE);
  Serial7.begin(SERIAL_BAUD_RATE);

  Wire.begin();

  motor.init();
}

void loop()
{
  // put your main code here, to run repeatedly:
  float theta = 0.0;
  while (true)
  {
    motor.cal_pwm(theta, 200);
    motor.move();
    theta += 0.1;
    if (theta > 2 * PI)
    {
      theta = 0.0;
    }
    delay(100);
  }
}
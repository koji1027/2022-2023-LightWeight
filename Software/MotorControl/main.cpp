#include "Arduino.h"
#include "src/motorcontrol.h"

void setup()
{
  // put your setup code here, to run once:
  motor_init();
}

void loop()
{
  // put your main code here, to run repeatedly:
  motor_control(0, 255);
  delay(1000);
  motor_control(PI / 2, 255);
}
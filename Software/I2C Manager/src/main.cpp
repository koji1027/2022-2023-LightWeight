#include <Arduino.h>
#include <Wire.h>
#include <bmx055.h>

#define SERIAL_BAUD 115200

Axis bmx055;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  bmx055.init();
  delay(2000);
  bmx055.accl_gyro_calibration();
  // Serial.println("Rotate the sensor");
  // bmx055.mag_calibration();
  delay(100);
}

void loop()
{
  // put your main code here, to run repeatedly:
  // bmx055.accl();
  // bmx055.gyro();
  bmx055.gyro();
  bmx055.show();
  delay(100);
}
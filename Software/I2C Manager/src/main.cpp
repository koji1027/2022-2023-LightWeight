#include <Arduino.h>
#include <Wire.h>
#include <bmx055.h>

#define SERIAL_BAUD 115200

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
}

void loop()
{
  // put your main code here, to run repeatedly:
}
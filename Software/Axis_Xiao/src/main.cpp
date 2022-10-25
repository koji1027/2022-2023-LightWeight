#include <Arduino.h>
#include <Wire.h>
#include <bmx055.h>

#define SERIAL_BAUD 115200

Axis bmx055;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(SERIAL_BAUD);
  Serial1.begin(SERIAL_BAUD);
  Wire.begin();
  delay(100);
  bmx055.init();
  delay(100);
}

void setup1(){
  Serial1.begin(SERIAL_BAUD);
}

void loop()
{
  bmx055.cal_angle();
  bmx055.show(false, false, false);
  }

void loop1(){
  if (Serial1.available())
  {
    int recv_data = Serial1.read();
    if (recv_data == 255)
    {
      bmx055.send();
    }
  }
  else{
    //Serial.println("Ouch!");
  }
  bmx055.send();
  delay(10);
}
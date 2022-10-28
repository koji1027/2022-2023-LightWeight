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
  bmx055.init();
}

void setup1(){
  Serial1.begin(SERIAL_BAUD);
  pinMode(A0, INPUT);
  analogReadResolution(10);
}

void loop()
{
  bmx055.cal();
  bmx055.show(true, false, false);
  }

void loop1(){
  if (Serial1.available() > 0)
  {
    int recv_data = Serial1.read();
    if (recv_data == 255) {
      bmx055.send();
      float battery_voltage = analogRead(A0);
      battery_voltage = battery_voltage * 3.3 / 1023.0;
      battery_voltage *= 4.0;
      int strech_battery_voltage = battery_voltage * 10;
      Serial1.write(strech_battery_voltage);
    }
  }
}
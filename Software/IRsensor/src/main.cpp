#include <Arduino.h>
#include <moving_average.h>
#include <ir_sensor.h>

IR ir;

void setup()
{
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(SIG_pin, INPUT);
  pinMode(A0, INPUT);

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  Serial.begin(115200);

  ir.begin();
}

void loop()
{
  while (0)
  {
    digitalWrite(s0, HIGH);
    digitalWrite(s1, HIGH);
    digitalWrite(s2, HIGH);
    digitalWrite(s3, HIGH);
    delayMicroseconds(10);
    int a = analogRead(A1);
    Serial.println(a);
  }
  ir.IR_get();
  // ir.IRpin_read();
  // ir.radius_read();
  //ir.angle_read();
}

void setup1()
{
  Serial1.begin(115200);
}

void loop1()
{
  /*while (!Serial1.available())
  {
  }*/
  int recv_data = Serial1.read();
  if (recv_data == 255)
  {
    ir.send();
  }
  // ir.IRpin_read();
  // ir.radius_read();
}
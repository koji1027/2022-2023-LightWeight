#include <Arduino.h>
#include <motor_control.h>

#define CONST_ANG_VEL 0.5

void ir_get();
void gyro_get();
void line_get();

int line_flag[32];
float ir_deg = 0;
float gyro_deg = 0;
const float LINE_ANGLE[32] = {
    1.0 * PI / 16.0,
    2.0 * PI / 16.0,
    3.0 * PI / 16.0,
    4.0 * PI / 16.0,
    5.0 * PI / 16.0,
    6.0 * PI / 16.0,
    7.0 * PI / 16.0,
    8.0 * PI / 16.0,
    9.0 * PI / 16.0,
    10.0 * PI / 16.0,
    11.0 * PI / 16.0,
    12.0 * PI / 16.0,
    13.0 * PI / 16.0,
    14.0 * PI / 16.0,
    15.0 * PI / 16.0,
    16.0 * PI / 16.0,
    -15.0 * PI / 16.0,
    -14.0 * PI / 16.0,
    -13.0 * PI / 16.0,
    -12.0 * PI / 16.0,
    -11.0 * PI / 16.0,
    -10.0 * PI / 16.0,
    -9.0 * PI / 16.0,
    -8.0 * PI / 16.0,
    -7.0 * PI / 16.0,
    -6.0 * PI / 16.0,
    -5.0 * PI / 16.0,
    -4.0 * PI / 16.0,
    -3.0 * PI / 16.0,
    -2.0 * PI / 16.0,
    -1.0 * PI / 16.0,
    0.0 * PI / 16.0,
};

float line[2] = {0.00, 0.00};

motor_control Motor;

void setup()
{
  // put your setup code here, to run once:
  // Serial.begin(115200);
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(9600);
  Serial5.begin(9600);
  Motor.begin();
  delay(2000);
}

void loop()
{
  // put your main code here, to run repeatedly:
  /*ir_get();
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
  delay(10);*/
  /*while (1)
  {
    while (!Serial4.available())
    {
    }
    Serial.println();
  }*/
  line_get();
  for (int i = 0; i < 8; i++)
  {
    Serial.print(i * 4);
    Serial.print(" : ");
    Serial.print(line_flag[i * 4]);
    Serial.print(" ");
    Serial.print(i * 4 + 1);
    Serial.print(" : ");
    Serial.print(line_flag[i * 4 + 1]);
    Serial.print(" ");
    Serial.print(i * 4 + 2);
    Serial.print(" : ");
    Serial.print(line_flag[i * 4 + 2]);
    Serial.print(" ");
    Serial.print(i * 4 + 3);
    Serial.print(" : ");
    Serial.print(line_flag[i * 4 + 3]);
    Serial.println();
  }
  float line_rad = atan2(line[1], line[0]);
  float line_deg = line_rad * 180.0 / PI;
  Serial.print("LINE_X : ");
  Serial.print(line[0]);
  Serial.print(" LINE_Y : ");
  Serial.print(line[1]);
  Serial.print(" LINE : ");
  Serial.println(line_deg);
  delay(300);
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

void line_get()
{
  line[0] = 0.00;
  line[1] = 0.00;
  float _line_flag[32];
  byte header = 254;
  Serial4.write(header);
  int recv_data = Serial4.read();
  while (!Serial4.available())
  {
  }
  if (recv_data == 255)
  {
    for (int i = 0; i < 16; i++)
    {
      while (!Serial4.available())
      {
      }
      recv_data = Serial4.read();
      _line_flag[i] = recv_data;
    }
  }
  Serial5.write(header);
  recv_data = Serial5.read();
  while (!Serial5.available())
  {
  }
  if (recv_data == 255)
  {
    for (int i = 16; i < 32; i++)
    {
      while (!Serial5.available())
      {
      }
      recv_data = Serial5.read();
      _line_flag[i] = recv_data;
    }
  }
  for (int i = 0; i < 32; i++)
  {
    int _i = i - 16 + 32;
    _i %= 32;
    line_flag[_i] = _line_flag[i];
  }
  for (int i = 0; i < 32; i++)
  {
    if (line_flag[i] == 1)
    {
      line[0] += cos(LINE_ANGLE[i]);
      line[1] += sin(LINE_ANGLE[i]);
      Serial.print("cos : ");
      Serial.print(cos(LINE_ANGLE[i]));
      Serial.print(" sin : ");
      Serial.println(sin(LINE_ANGLE[i]));
    }
  }
}
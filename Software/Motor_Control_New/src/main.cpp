#include <Arduino.h>
#include <motor_control.h>

#define CONST_ANG_VEL 0.5
void ir_get();
void gyro_get();
void line_get();

int line_flag[32];
bool line_whole_flag;
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
  Serial4.begin(115200);
  Serial5.begin(115200);
  Motor.begin();
  delay(2000);
}

void loop()
{
  gyro_get();
  ir_get();
  float ir_rad = ir_deg * PI / 180;
  line_get();
  float line_rad = atan2(line[1], line[0]);
  float line_deg = line_rad * 180.0 / PI;
  Motor.cal(ir_deg, 150, 0, gyro_deg);
  Serial.println(ir_deg);
  //delay(100);
}

void ir_get()
{
  Serial2.write(255);
  while (!Serial2.available())
  {
    // Serial.println("ir_Ouch!");
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
    // Serial.println("gyro_Ouch!");
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
  line_whole_flag = 0;
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
      // Serial.print("cos : ");
      // Serial.print(cos(LINE_ANGLE[i]));
      // Serial.print(" sin : ");
      // Serial.println(sin(LINE_ANGLE[i]));
      line_whole_flag = 1;
    }
  }
}
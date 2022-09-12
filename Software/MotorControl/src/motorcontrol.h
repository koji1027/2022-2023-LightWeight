#include <Arduino.h>
#include <math.h>
#include <BMX055.h>

#define MOTOR_NUM 4
#define MOTOR_PIN 2

const int MOTORS_PIN[MOTOR_NUM][MOTOR_PIN] = {{1, 2}, {5, 6}, {9, 10}, {13, 14}};              // PWMピン、DIRピン
const double MOTORS_THETA[MOTOR_NUM] = {PI / 3.0, 2.0 * PI / 3.0, -PI / 3.0, -2.0 * PI / 3.0}; //モータの配置角度(ラジアン) 正面０ラジアン、-PI < theta <= PI

float degree = 0.00;
float radian;
float prezGyro = 0.00;
unsigned long long preMicros = 0;

void motor_move(double motor_power[MOTOR_NUM], int motor_dir[MOTOR_NUM])
{
  /*for (int i = 0; i < MOTOR_NUM; i++) {
    motor_dir[i] = !motor_dir[i];
  }*/
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    analogWrite(MOTORS_PIN[i][0], int(motor_power[i]));
    digitalWrite(MOTORS_PIN[i][1], motor_dir[i]);
  }
}

int get_max_index(double data[], int data_num)
{
  int max_index = 0;
  int max = 0;
  for (int i = 0; i < data_num; i++)
  {
    if (data[i] > max)
    {
      max = data[i];
      max_index = i;
    }
  }
  return max_index;
}

void motor_init()
{
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    pinMode(MOTORS_PIN[i][0], OUTPUT);
    pinMode(MOTORS_PIN[i][1], OUTPUT);
  }
}

void motor_control(double theta, uint8_t power)
{ //パワーは、0 ~ 255
  double motor_power[MOTOR_NUM] = {0, 0, 0, 0};
  int motor_dir[MOTOR_NUM] = {1, 1, 1, 1}; // 0:逆転 1:正転
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    motor_power[i] = double(power) * sin(theta - MOTORS_THETA[i]);
  }
  int max_index = get_max_index(motor_power, MOTOR_NUM);
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    motor_power[i] = round(motor_power[i] / motor_power[max_index] * double(power));
  }
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    if (motor_power[i] < 0)
    {
      motor_dir[i] = 1;
      motor_power[i] = -motor_power[i];
    }
    else {
      motor_dir[i] = 0;
    }
  }
  motor_move(motor_power, motor_dir);
}

void gyro_posture_spin(uint8_t spinpower){
  double spin_power[MOTOR_NUM]={spinpower,spinpower,spinpower,spinpower};
  int rightspin_dir[MOTOR_NUM] = {1,1,0,0};
  int leftspin_dir[MOTOR_NUM] = {0,0,1,1};
  BMX055_Gyro();
  zGyro += 0.01;
  unsigned long long time = micros();
  degree += (zGyro + prezGyro) * float(time - preMicros) / 2000000;
  radian = -degree * PI / 180;
  preMicros = time;
  prezGyro = zGyro;

  if(radian > 0){
    motor_move(spin_power,rightspin_dir);
  }
  if(radian < 0){
    motor_move(spin_power,leftspin_dir);
  }
}
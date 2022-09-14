#include <Arduino.h>
#include <math.h>
#include <BMX055.h>

#define MOTOR_NUM 4
#define MOTOR_PIN 2
//#define SPIN_ADJUST 150 Gyro
#define SPIN_ADJUST 70

int mag_posture_spin();

const int MOTORS_PIN[MOTOR_NUM][MOTOR_PIN] = {{1, 2}, {5, 6}, {9, 10}, {13, 14}};              // PWMピン、DIRピン
const double MOTORS_THETA[MOTOR_NUM] = {PI / 3.0, 2.0 * PI / 3.0, -PI / 3.0, -2.0 * PI / 3.0}; //モータの配置角度(ラジアン) 正面０ラジアン、-PI < theta <= PI


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
    if (abs(data[i]) > max)
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
  /*int adjust_power = mag_posture_spin();
  for (int i = 0; i < MOTOR_NUM; i++) {
    motor_power[i] += adjust_power;
  }*/

  int max_index = get_max_index(motor_power, MOTOR_NUM);
  for (int i = 0; i < MOTOR_NUM; i++) {
    motor_power[i] = round(float(power) * (motor_power[i] / abs(motor_power[max_index]))); 
  }
  
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    if (motor_power[i] < 0)
    {
      motor_dir[i] = 1;
      motor_power[i] = -motor_power[i];
    }
    else
    {
      motor_dir[i] = 0;
    }
  }
  motor_move(motor_power, motor_dir);
}

void gyro_posture_spin()
{
  uint8_t spinpower = 0;
  int rightspin_dir[MOTOR_NUM] = {1, 1, 1, 1};                                 //回転は、すべてのタイヤが同じ方向に回るから修正しといた
  int leftspin_dir[MOTOR_NUM] = {0, 0, 0, 0};                                  //回転は、すべてのタイヤが同じ方向に回るから修正しといた
  radian_g = BMX055_Gyro();
  Serial.println(radian_g);
  int _power = abs(radian_g)*SPIN_ADJUST;
  if (_power > 255){
    spinpower = 255;
  }
  else {
    spinpower = _power;
  }
  double spin_power[MOTOR_NUM] = {spinpower, spinpower, spinpower, spinpower}; //回転のちからは、角度のズレにおおじて計算したほうがいいともう ex) （ズレ） * 1 的な...
  Serial.print("radian:");
  Serial.print(radian_g);
  Serial.print(" power:");
  Serial.println(spinpower);
  if (radian_g > PI / 80.0) //ぴったり０ラジアンにすると振動するから余裕を持たせる
  {
    motor_move(spin_power, rightspin_dir);
  }
  if (radian_g < -PI / 80.0)
  {
    motor_move(spin_power, leftspin_dir);
  }
  /*BMX055_Gyro();
  zGyro -= 0.01;
  unsigned long long time = micros();
  degree += (zGyro + prezGyro) * float(time - preMicros) / 2000000;
  preMicros = time;
  prezGyro = zGyro;
  Serial.println(xGyro);
  //Serial.println("°");*/
}

int mag_posture_spin()
{
  uint8_t spinpower = 0;
  int rightspin_dir[MOTOR_NUM] = {1, 1, 1, 1};                                 //回転は、すべてのタイヤが同じ方向に回るから修正しといた
  int leftspin_dir[MOTOR_NUM] = {0, 0, 0, 0};                                  //回転は、すべてのタイヤが同じ方向に回るから修正しといた
  radian_m = BMX055_Mag();
  //Serial.println(radian_m);
  int _power = abs(radian_m)*SPIN_ADJUST;
  if (_power > 255){
    spinpower = 255;
  }
  else {
    spinpower = _power;
  }
  //double spin_power[MOTOR_NUM] = {spinpower, spinpower, spinpower, spinpower}; //回転のちからは、角度のズレにおおじて計算したほうがいいともう ex) （ズレ） * 1 的な...
  Serial.print("radian:");
  Serial.print(radian_m);
  Serial.print(" power:");
  Serial.println(spinpower);
  /*if (radian_m > PI / 80.0) //ぴったり０ラジアンにすると振動するから余裕を持たせる
  {
    motor_move(spin_power, rightspin_dir);
  }
  if (radian_m < -PI / 80.0)
  {
    motor_move(spin_power, leftspin_dir);
  }*/
  return spinpower;
}

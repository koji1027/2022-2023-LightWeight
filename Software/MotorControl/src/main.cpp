#include <Arduino.h>
#include <motorcontrol.h>
#include <math.h>
//#include <BMX055.h>

#define MOTOR_NUM 4
#define MOTOR_PIN 2

// const int MOTORS_PIN[MOTOR_NUM][MOTOR_PIN] = {{1, 2}, {5, 6}, {9, 10}, {13, 14}};              // PWMピン、DIRピン
// const double MOTORS_THETA[MOTOR_NUM] = {PI / 3.0, 2.0 * PI / 3.0, -2.0 * PI / 3.0, -PI / 3.0}; //モータの配置角度(ラジアン) 正面０ラジアン、-PI < theta <= PI

void motor_control(double theta, uint8_t power);
void motor_move(double motor_power[MOTOR_NUM], int motor_dir[MOTOR_NUM]);
int get_max_index(double data[], int data_num);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);
  Serial6.begin(115200);
  Serial7.begin(115200);

  // Wire.begin();
  // BMX055_Init();

  for (int i = 0; i < MOTOR_NUM; i++)
  {
    pinMode(MOTORS_PIN[i][0], OUTPUT);
    pinMode(MOTORS_PIN[i][1], OUTPUT);
  }

  digitalWrite(LED_BUILTIN,1);
}

void loop()
{
  // put your main code here, to run repeatedly:
  uint8_t power = 255;
  double theta = 0;
  /*while (true)
  {
    motor_control(theta, power);
    //delay();
  }*/
  /*while (1)
  {
    digitalWrite(3, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(18, HIGH);
    digitalWrite(2, HIGH);
    digitalWrite(9, HIGH);
    digitalWrite(11, HIGH);
    digitalWrite(13, HIGH);
    /*analogWrite(2, 120);
    analogWrite(9, 120);
    analogWrite(11, 120);
    analogWrite(13, 120);
  }*/
  while(1){
    motor_control(theta, power);
    theta++;
    theta = theta
  }
}
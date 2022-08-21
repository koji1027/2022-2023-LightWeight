#include <Arduino.h>
#include <math.h>

#define COM A0
#define THRESHOLD 500
#define SENSOR_NUM 16
#define KICKER_PIN D5
#define LED_PIN D10
#define SELECT_PIN0 D1
#define SELECT_PIN1 D2
#define SELECT_PIN2 D3
#define SELECT_PIN3 D4

const float SENSOR_THETA[SENSOR_NUM] = 
{
  PI, -PI*15.0/16.0, -PI*14.0/16.0, -PI*13.0/16.0, -PI*12.0/16.0, -PI*11.0/16.0, -PI*10.0/16.0, -PI*9.0/16.0,
  -PI*8.0/16.0, -PI*7.0/16.0, -PI*6.0/16.0, -PI*5.0/16.0, -PI*4.0/16.0, -PI*3.0/16.0, -PI*2.0/16.0, -PI*1.0/16.0
};

float sensor_x[SENSOR_NUM] = {};
float sensor_y[SENSOR_NUM] = {};
uint8_t mode = 0;
int line_flag[SENSOR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int sensor_value[SENSOR_NUM] = {};
float line_x = 0;
float line_y = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(SELECT_PIN0, OUTPUT);
  pinMode(SELECT_PIN1, OUTPUT);
  pinMode(SELECT_PIN2, OUTPUT);
  pinMode(SELECT_PIN3, OUTPUT);

  digitalWrite(SELECT_PIN0, LOW);
  digitalWrite(SELECT_PIN1, LOW);
  digitalWrite(SELECT_PIN2, LOW);
  digitalWrite(SELECT_PIN3, LOW);

  for (int i = 0;i < SENSOR_NUM;i++) {
    sensor_x[i] = cos(SENSOR_THETA[i]);
    sensor_y[i] = sin(SENSOR_THETA[i]);
  }
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int _sensor_value[SENSOR_NUM];
  int _line_flag[SENSOR_NUM];
  for (int i =0; i < SENSOR_NUM; i++) {
    digitalWrite(SELECT_PIN0, byte(i) & (1 << 0));
    digitalWrite(SELECT_PIN1, byte(i) & (1 << 1));
    digitalWrite(SELECT_PIN2, byte(i) & (1 << 2));
    digitalWrite(SELECT_PIN3, byte(i) & (1 << 3));
    _sensor_value[i] = analogRead(COM);
  }
  sensor_value[8] = _sensor_value[0];
  sensor_value[9] = _sensor_value[1];
  sensor_value[10] = _sensor_value[2];
  sensor_value[11] = _sensor_value[3];
  sensor_value[12] = _sensor_value[4];
  sensor_value[13] = _sensor_value[5];
  sensor_value[14] = _sensor_value[6];
  sensor_value[15] = _sensor_value[7];
  sensor_value[0] = _sensor_value[8];
  sensor_value[1] = _sensor_value[9];
  sensor_value[2] = _sensor_value[10];
  sensor_value[3] = _sensor_value[11];
  sensor_value[4] = _sensor_value[12];
  sensor_value[5] = _sensor_value[13];
  sensor_value[6] = _sensor_value[14];
  sensor_value[7] = _sensor_value[15];

  for (int i = 0; i < SENSOR_NUM; i++) {
    if (sensor_value[i] > THRESHOLD) {
      line_flag[i] = 1;
    } else {
      line_flag[i] = 0;
    }
  }
  
  for (int i = 0; i < SENSOR_NUM; i++) {
    if (line_flag[i] == 1) {
      line_x += sensor_x[i] * line_flag[i];
      line_y += sensor_y[i] * line_flag[i];
    }
  }
  for (int i = 0;i < SENSOR_NUM;i++) {
    Serial.print(sensor_value[i]);
    Serial.print(" ");
  }
  Serial.println("");
  delay(50);
}

void setup1() {
  pinMode(KICKER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  Serial1.begin(115200);
}

void loop1() {

}
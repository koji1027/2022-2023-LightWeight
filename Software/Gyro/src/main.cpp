#include "Arduino.h"
#include <math.h>
#include <Wire.h>

//================================================================//
//  AE-BMX055             Arduino UNO                             //
//    VCC                    +5V                                  //
//    GND                    GND                                  //
//    SDA                    A4(SDA)                              //
//    SCL                    A5(SCL)                              //
//                                                                //
//   (JP4,JP5,JP6はショートした状態)                                //
//   http://akizukidenshi.com/catalog/g/gK-13010/                 //
//================================================================//

#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)
#define PRINT_RATE 40 
#define ANGULAR_SENSITYVITY 0.00875 

// センサーの値を保存するグローバル変数
float zGyro = 0.00;
double psi = 0;
float Gz;

double phi = 0;
double theta = 0;

unsigned long InitTime, RunTime, PrintTime, sample;
uint8_t PrintRate = 1000/PRINT_RATE;

void BMX055_Init();
void BMX055_Gyro();

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  BMX055_Init();
  InitTime = millis();
  sample = 0;
}

void loop()
{
  /*Serial.println("--------------------------------------");
  Serial.print("Gyro= ");
  Serial.print(xGyro);
  Serial.print(",");
  Serial.print(yGyro);
  Serial.print(",");
  Serial.print(zGyro);
  Serial.println(""); 
  delay(1000);*/

  BMX055_Gyro();
  RunTime = millis()-InitTime;

  if(((millis()-PrintTime)>PrintRate)&&(zGyro<10000))
  {
    Gz = zGyro*ANGULAR_SENSITYVITY*PI/180;
    psi = psi+zGyro * (millis()-PrintTime)/1000;
    Serial.print("zGyro:\t");
    Serial.println(zGyro);
    Serial.print("\tpsi:\t");
    Serial.println((int)(psi*180/PI));
    PrintTime = millis();
    sample++;
  }

  delay(500);
}

//=====================================================================================//
void BMX055_Init()
{
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x04);  // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);  // Select Bandwidth register
  Wire.write(0x07);  // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
 
}

void BMX055_Gyro()
{
  unsigned int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)  zGyro -= 65536;

  zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s
}
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>

Madgwick MadgwickFilter;
MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

void setup() 
{
  Serial.begin(115200);
  MadgwickFilter.begin(200); //100Hz
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find MPU6050 sensor");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
}

void loop()
{
  timer = millis();

  // Read normalized values
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + gyr.YAxis * timeStep;
  roll = roll + gyr.XAxis * timeStep;
  yaw = yaw + gyr.ZAxis * timeStep;
  MadgwickFilter.updateIMU(gyr.XAxis, gyr.YAxis, gyr.ZAxis, acc.XAxis, acc.YAxis, acc.ZAxis);
  // Output raw
  Serial.print("補正前 Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);  
  Serial.print(" Yaw = ");
  Serial.println(yaw);
  
  Serial.print("補正後 Pitch = ");
  Serial.print(MadgwickFilter.getPitch());
  Serial.print(" Roll = ");
  Serial.print(MadgwickFilter.getRoll());
  Serial.print(" Yaw = ");
  Serial.println(MadgwickFilter.getYaw());

  // Wait to full timeStep period
  delay((timeStep * 1000) - (millis() - timer));
}


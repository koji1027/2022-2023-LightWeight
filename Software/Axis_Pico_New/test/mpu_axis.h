#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)



class MPU_Axis
{
public:
    // IMU_Zero.cpp//
    void zero_reader(); //作った
    void ForceHeader();
    void GetSmoothed();
    void Initialize();
    void SetOffsets(int TheOffsets[6]);
    void ShowProgress();
    void PullBracketsIn();
    void PullBracketsOut();
    void SetAveraging(int NewN);

    // IMU_Zero.cpp//
    void DMP_init();
    void DMP_read();
    void dmpDataReady();

private:
    ////////////////
    //IMU_Zero.cpp//
    ////////////////

    const char LBRACKET = '[';
    const char RBRACKET = ']';
    const char COMMA = ',';
    const char BLANK = ' ';
    const char PERIOD = '.';

    const int iAx = 0;
    const int iAy = 1;
    const int iAz = 2;
    const int iGx = 3;
    const int iGy = 4;
    const int iGz = 5;

    const int usDelay = 3150; // empirical, to hold sampling to 200 Hz
    const int NFast = 1000;   // the bigger, the better (but slower)
    const int NSlow = 10000;  // ..
    const int LinesBetweenHeaders = 5;
    int LowValue[6];
    int HighValue[6];
    int Smoothed[6];
    int LowOffset[6];
    int HighOffset[6];
    int Target[6];
    int LinesOut;
    int N;

    ////////////////////
    //MPU6050_DMP6.cpp//
    ////////////////////

    bool blinkState = false;

    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;        // [w, x, y, z]         quaternion container
    VectorInt16 aa;      // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity; // [x, y, z]            gravity vector
    float euler[3];      // [psi, theta, phi]    Euler angle container
    float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    // packet structure for InvenSense teapot demo
    uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

    volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
};
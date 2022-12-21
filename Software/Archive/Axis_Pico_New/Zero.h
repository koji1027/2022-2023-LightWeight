#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA    = ',';
const char BLANK    = ' ';
const char PERIOD   = '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150;   // empirical, to hold sampling to 200 Hz
const int NFast =  1000;    // the bigger, the better (but slower)
const int NSlow = 10000;    // ..
const int LinesBetweenHeaders = 5;
      int LowValue[6];
      int HighValue[6];
      int Smoothed[6];
      int LowOffset[6];
      int HighOffset[6];
      int Target[6];
      int LinesOut;
      int N;

void ZeroReader();

void ForceHeader();
void GetSmoothed();
void Initialize();
void SetOffsets(int TheOffsets[6]);
void ShowProgress();
void PullBracketsIn();
void PullBracketsOut();
void SetAveraging(int NewN);
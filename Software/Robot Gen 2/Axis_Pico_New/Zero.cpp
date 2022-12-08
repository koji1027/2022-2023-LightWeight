#include "Zero.h"

MPU6050 accelgyro;

void ZeroReader()
{
    Initialize();
    for (int i = iAx; i <= iGz; i++)
    {                  // set targets and initial guesses
        Target[i] = 0; // must fix for ZAccel
        HighOffset[i] = 0;
        LowOffset[i] = 0;
    } // set targets and initial guesses
    Target[iAz] = 16384;
    SetAveraging(NFast);

    PullBracketsOut();
    PullBracketsIn();

    Serial.println("-------------- done --------------");
}

void ForceHeader()
{
    LinesOut = 99;
}

void GetSmoothed()
{
    int16_t RawValue[6];
    int i;
    long Sums[6];
    for (i = iAx; i <= iGz; i++)
    {
        Sums[i] = 0;
    }
    //    unsigned long Start = micros();

    for (i = 1; i <= N; i++)
    { // get sums
        accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz],
                             &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
        if ((i % 500) == 0)
            Serial.print(PERIOD);
        delayMicroseconds(usDelay);
        for (int j = iAx; j <= iGz; j++)
            Sums[j] = Sums[j] + RawValue[j];
    } // get sums
      //    unsigned long usForN = micros() - Start;
      //    Serial.print(" reading at ");
      //    Serial.print(1000000/((usForN+N/2)/N));
      //    Serial.println(" Hz");
    for (i = iAx; i <= iGz; i++)
    {
        Smoothed[i] = (Sums[i] + N / 2) / N;
    }
} // GetSmoothed

void Initialize()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println("PID tuning Each Dot = 100 readings");
    /*A tidbit on how PID (PI actually) tuning works.
      When we change the offset in the MPU6050 we can get instant results. This allows us to use Proportional and
      integral of the PID to discover the ideal offsets. Integral is the key to discovering these offsets, Integral
      uses the error from set-point (set-point is zero), it takes a fraction of this error (error * ki) and adds it
      to the integral value. Each reading narrows the error down to the desired offset. The greater the error from
      set-point, the more we adjust the integral value. The proportional does its part by hiding the noise from the
      integral math. The Derivative is not used because of the noise and because the sensor is stationary. With the
      noise removed the integral value lands on a solid offset after just 600 readings. At the end of each set of 100
      readings, the integral value is used for the actual offsets and the last proportional reading is ignored due to
      the fact it reacts to any noise.
    */
    accelgyro.CalibrateAccel(6);
    accelgyro.CalibrateGyro(6);
    Serial.println("\nat 600 Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println();
    accelgyro.CalibrateAccel(1);
    accelgyro.CalibrateGyro(1);
    Serial.println("700 Total Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println();
    accelgyro.CalibrateAccel(1);
    accelgyro.CalibrateGyro(1);
    Serial.println("800 Total Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println();
    accelgyro.CalibrateAccel(1);
    accelgyro.CalibrateGyro(1);
    Serial.println("900 Total Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println();
    accelgyro.CalibrateAccel(1);
    accelgyro.CalibrateGyro(1);
    Serial.println("1000 Total Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:");
} // Initialize

void SetOffsets(int TheOffsets[6])
{
    accelgyro.setXAccelOffset(TheOffsets[iAx]);
    accelgyro.setYAccelOffset(TheOffsets[iAy]);
    accelgyro.setZAccelOffset(TheOffsets[iAz]);
    accelgyro.setXGyroOffset(TheOffsets[iGx]);
    accelgyro.setYGyroOffset(TheOffsets[iGy]);
    accelgyro.setZGyroOffset(TheOffsets[iGz]);
} // SetOffsets

void ShowProgress()
{
    if (LinesOut >= LinesBetweenHeaders)
    { // show header
        Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
        LinesOut = 0;
    } // show header
    Serial.print(BLANK);
    for (int i = iAx; i <= iGz; i++)
    {
        Serial.print(LBRACKET);
        Serial.print(LowOffset[i]),
            Serial.print(COMMA);
        Serial.print(HighOffset[i]);
        Serial.print("] --> [");
        Serial.print(LowValue[i]);
        Serial.print(COMMA);
        Serial.print(HighValue[i]);
        if (i == iGz)
        {
            Serial.println(RBRACKET);
        }
        else
        {
            Serial.print("]\t");
        }
    }
    LinesOut++;
} // ShowProgress

void PullBracketsIn()
{
    boolean AllBracketsNarrow;
    boolean StillWorking;
    int NewOffset[6];

    Serial.println("\nclosing in:");
    AllBracketsNarrow = false;
    ForceHeader();
    StillWorking = true;
    while (StillWorking)
    {
        StillWorking = false;
        if (AllBracketsNarrow && (N == NFast))
        {
            SetAveraging(NSlow);
        }
        else
        {
            AllBracketsNarrow = true;
        } // tentative
        for (int i = iAx; i <= iGz; i++)
        {
            if (HighOffset[i] <= (LowOffset[i] + 1))
            {
                NewOffset[i] = LowOffset[i];
            }
            else
            { // binary search
                StillWorking = true;
                NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
                if (HighOffset[i] > (LowOffset[i] + 10))
                {
                    AllBracketsNarrow = false;
                }
            } // binary search
        }
        SetOffsets(NewOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
        { // closing in
            if (Smoothed[i] > Target[i])
            { // use lower half
                HighOffset[i] = NewOffset[i];
                HighValue[i] = Smoothed[i];
            } // use lower half
            else
            { // use upper half
                LowOffset[i] = NewOffset[i];
                LowValue[i] = Smoothed[i];
            } // use upper half
        }     // closing in
        ShowProgress();
    } // still working

} // PullBracketsIn

void PullBracketsOut()
{
    boolean Done = false;
    int NextLowOffset[6];
    int NextHighOffset[6];

    Serial.println("expanding:");
    ForceHeader();

    while (!Done)
    {
        Done = true;
        SetOffsets(LowOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
        { // got low values
            LowValue[i] = Smoothed[i];
            if (LowValue[i] >= Target[i])
            {
                Done = false;
                NextLowOffset[i] = LowOffset[i] - 1000;
            }
            else
            {
                NextLowOffset[i] = LowOffset[i];
            }
        } // got low values

        SetOffsets(HighOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
        { // got high values
            HighValue[i] = Smoothed[i];
            if (HighValue[i] <= Target[i])
            {
                Done = false;
                NextHighOffset[i] = HighOffset[i] + 1000;
            }
            else
            {
                NextHighOffset[i] = HighOffset[i];
            }
        } // got high values
        ShowProgress();
        for (int i = iAx; i <= iGz; i++)
        {
            LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
            HighOffset[i] = NextHighOffset[i]; // ..
        }
    } // keep going
} // PullBracketsOut

void SetAveraging(int NewN)
{
    N = NewN;
    Serial.print("averaging ");
    Serial.print(N);
    Serial.println(" readings each time");
} // SetAveraging
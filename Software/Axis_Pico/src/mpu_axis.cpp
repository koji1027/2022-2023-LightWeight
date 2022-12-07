#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#include "mpu_axis.h"
#include "MPU6050_6Axis_MotionApps20.h"

////////////////
//IMU_Zero.cpp//
////////////////
void MPU_Axis::set_MPU(MPU6050 *_mpu){
    p_mpu = _mpu;
}

void MPU_Axis::zero_reader()
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

void MPU_Axis::ForceHeader()
{
    LinesOut = 99;
}

void MPU_Axis::GetSmoothed()
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
        p_mpu->getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz],
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

void MPU_Axis::Initialize()
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
    p_mpu->initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(p_mpu->testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
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
    p_mpu->CalibrateAccel(6);
    p_mpu->CalibrateGyro(6);
    Serial.println("\nat 600 Readings");
    p_mpu->PrintActiveOffsets();
    Serial.println();
    p_mpu->CalibrateAccel(1);
    p_mpu->CalibrateGyro(1);
    Serial.println("700 Total Readings");
    p_mpu->PrintActiveOffsets();
    Serial.println();
    p_mpu->CalibrateAccel(1);
    p_mpu->CalibrateGyro(1);
    Serial.println("800 Total Readings");
    p_mpu->PrintActiveOffsets();
    Serial.println();
    p_mpu->CalibrateAccel(1);
    p_mpu->CalibrateGyro(1);
    Serial.println("900 Total Readings");
    p_mpu->PrintActiveOffsets();
    Serial.println();
    p_mpu->CalibrateAccel(1);
    p_mpu->CalibrateGyro(1);
    Serial.println("1000 Total Readings");
    p_mpu->PrintActiveOffsets();
    Serial.println("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:");
} // Initialize

void MPU_Axis::SetOffsets(int TheOffsets[6])
{
    p_mpu->setXAccelOffset(TheOffsets[iAx]);
    p_mpu->setYAccelOffset(TheOffsets[iAy]);
    p_mpu->setZAccelOffset(TheOffsets[iAz]);
    p_mpu->setXGyroOffset(TheOffsets[iGx]);
    p_mpu->setYGyroOffset(TheOffsets[iGy]);
    p_mpu->setZGyroOffset(TheOffsets[iGz]);
} // SetOffsets

void MPU_Axis::ShowProgress()
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

void MPU_Axis::PullBracketsIn()
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

void MPU_Axis::PullBracketsOut()
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

void MPU_Axis::SetAveraging(int NewN)
{
    N = NewN;
    Serial.print("averaging ");
    Serial.print(N);
    Serial.println(" readings each time");
} // SetAveraging

////////////////////
//MPU6050_DMP6.cpp//
////////////////////

void MPU_Axis::DMP_init(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    p_mpu->initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(p_mpu->testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = p_mpu->dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    p_mpu->setXGyroOffset(220);
    p_mpu->setYGyroOffset(76);
    p_mpu->setZGyroOffset(-85);
    p_mpu->setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        p_mpu->CalibrateAccel(6);
        p_mpu->CalibrateGyro(6);
        p_mpu->PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        p_mpu->setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = p_mpu->getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = p_mpu->dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void MPU_Axis::DMP_read(){
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (p_mpu->dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            p_mpu->dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            p_mpu->dmpGetQuaternion(&q, fifoBuffer);
            p_mpu->dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            p_mpu->dmpGetQuaternion(&q, fifoBuffer);
            p_mpu->dmpGetGravity(&gravity, &q);
            p_mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            p_mpu->dmpGetQuaternion(&q, fifoBuffer);
            p_mpu->dmpGetAccel(&aa, fifoBuffer);
            p_mpu->dmpGetGravity(&gravity, &q);
            p_mpu->dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            p_mpu->dmpGetQuaternion(&q, fifoBuffer);
            p_mpu->dmpGetAccel(&aa, fifoBuffer);
            p_mpu->dmpGetGravity(&gravity, &q);
            p_mpu->dmpGetLinearAccel(&aaReal, &aa, &gravity);
            p_mpu->dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void MPU_Axis::dmpDataReady() {
    mpuInterrupt = true;
}
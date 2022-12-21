#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

class Gyro {
   public:
    void begin();
    void getEuler();
    void cal_vel();
    float angle = 0.0;
    float vel_x = 0.0;
    float vel_y = 0.0;

   private:
    float pre_ax = 0.0;
    float pre_ay = 0.0;
    unsigned long long pre_time = 0;
    bool blinkState = false;

    bool dmpReady = false;
    uint8_t mpuIntStatus;
    uint8_t devStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];

    Quaternion q;
    VectorInt16 aa;
    VectorInt16 gy;
    VectorInt16 aaReal;
    VectorInt16 aaWorld;
    VectorFloat gravity;
    float euler[3];
    float ypr[3];
    int16_t accelRaw[3];
    int16_t gyroRaw[3];
    float accel[3];

    uint8_t teapotPacket[14] = {'$', 0x02, 0, 0,    0,    0,    0,
                                0,   0,    0, 0x00, 0x00, '\r', '\n'};

    volatile bool mpuInterrupt = false;
    void dmpDataReady() { mpuInterrupt = true; }
};

void Gyro::begin() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(-2238);
    mpu.setYGyroOffset(352);
    mpu.setZGyroOffset(716);
    mpu.setXAccelOffset(242);
    mpu.setYAccelOffset(-196);
    mpu.setZAccelOffset(-26);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println();
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void Gyro::getEuler() {
    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))

#ifdef OUTPUT_READABLE_QUATERNION
        mpu.dmpGetQuaternion(&q, fifoBuffer);
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
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angle = ypr[0];
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++;
#endif
    mpu.getMotion6(&accelRaw[0], &accelRaw[1], &accelRaw[2], &gyroRaw[0],
                   &gyroRaw[1], &gyroRaw[2]);
    accel[0] = accelRaw[0] / 16384.0;
    accel[1] = accelRaw[1] / 16384.0;
    accel[2] = accelRaw[2] / 16384.0;
}

void Gyro::cal_vel() {
    unsigned long long now = micros();
    float dt = (now - pre_time) / 1000000.0;
    pre_time = now;
    vel_x -= (accel[0] + pre_ax) / 2 * dt;
    vel_y += (accel[1] + pre_ay) / 2 * dt;
    pre_ax = accel[0];
    pre_ay = accel[1];
    float theta = atan2(vel_y, vel_x);
    Serial.print("vel_x: ");
    Serial.print(vel_x);
    Serial.print("\tvel_y: ");
    Serial.print(vel_y);
    Serial.print("\ttheta: ");
    Serial.println(theta * 180 / PI);
}
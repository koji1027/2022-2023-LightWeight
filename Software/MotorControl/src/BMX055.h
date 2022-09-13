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
#include <Arduino.h>
#include <Wire.h>
// BMX055 加速度センサのI2Cアドレス
#define Addr_Accl 0x19 // (JP1,JP2,JP3 = Openの時)
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69 // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13 // (JP1,JP2,JP3 = Openの時)

// センサーの値を保存するグローバル変数
float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
float xGyro_offset = 0.00;
float yGyro_offset = 0.00;
float zGyro_offset = 0.00;
int xMag = 0;
int yMag = 0;
int zMag = 0;

float degree = 0.00;
float radian;
float prezGyro = 0.00;
unsigned long long preMicros = 0;

float BMX055_Gyro();
void BMX055_Mag();

void BMX055_Init()
{
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x0F); // Select PMU_Range register
    Wire.write(0x03); // Range = +/- 2g
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x10); // Select PMU_BW register
    Wire.write(0x08); // Bandwidth = 7.81 Hz
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x11); // Select PMU_LPW register
    Wire.write(0x00); // Normal mode, Sleep duration = 0.5ms
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x0F); // Select Range register
    Wire.write(0x00); // Full scale = +/- 2000 degree/s
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x10); // Select Bandwidth register
    Wire.write(0x07); // ODR = 100 Hz
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x11); // Select LPM1 register
    Wire.write(0x00); // Normal mode, Sleep duration = 2ms
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4B); // Select Mag register
    Wire.write(0x83); // Soft reset
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4B); // Select Mag register
    Wire.write(0x01); // Soft reset
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4C); // Select Mag register
    Wire.write(0x00); // Normal Mode, ODR = 10 Hz
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4E); // Select Mag register
    Wire.write(0x84); // X, Y, Z-Axis enabled
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x51); // Select Mag register
    Wire.write(0x04); // No. of Repetitions for X-Y Axis = 9
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x52); // Select Mag register
    Wire.write(0x16); // No. of Repetitions for Z-Axis = 15
    Wire.endTransmission();

    float x_offset = 0.00;
    float y_offset = 0.00;
    float z_offset = 0.00;
    for (int i = 0; i < 5; i++)
    {
        BMX055_Gyro();
        x_offset += xGyro;
        y_offset += yGyro;
        z_offset += zGyro;
        delay(100);
    }
    xGyro_offset = x_offset / 5.0;
    yGyro_offset = y_offset / 5.0;
    zGyro_offset = z_offset / 5.0;
}
//=====================================================================================//
void BMX055_Accl()
{
    unsigned int data[6];
    for (int i = 0; i < 6; i++)
    {
        Wire.beginTransmission(Addr_Accl);
        Wire.write((2 + i)); // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_Accl, 1); // Request 1 byte of data
        // Read 6 bytes of data
        // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data to 12-bits
    xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
    if (xAccl > 2047)
        xAccl -= 4096;
    yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
    if (yAccl > 2047)
        yAccl -= 4096;
    zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
    if (zAccl > 2047)
        zAccl -= 4096;
    xAccl = xAccl * 0.0098; // range = +/-2g
    yAccl = yAccl * 0.0098; // range = +/-2g
    zAccl = zAccl * 0.0098; // range = +/-2g
}
//=====================================================================================//
float BMX055_Gyro()
{
    unsigned int data[6];
    for (int i = 0; i < 6; i++)
    {
        Wire.beginTransmission(Addr_Gyro);
        Wire.write((2 + i)); // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_Gyro, 1); // Request 1 byte of data
        // Read 6 bytes of data
        // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data
    xGyro = (data[1] * 256) + data[0];
    if (xGyro > 32767)
        xGyro -= 65536;
    yGyro = (data[3] * 256) + data[2];
    if (yGyro > 32767)
        yGyro -= 65536;
    zGyro = (data[5] * 256) + data[4];
    if (zGyro > 32767)
        zGyro -= 65536;

    xGyro = xGyro * 0.06103515625; //  Full scale = +/- 125 degree/s
    yGyro = yGyro * 0.06103515625; //  Full scale = +/- 125 degree/s
    zGyro = zGyro * 0.06103515625; //  Full scale = +/- 125 degree/s
    xGyro -= xGyro_offset;
    yGyro -= yGyro_offset;
    zGyro -= zGyro_offset;

    zGyro += 0.01;
    unsigned long long time = micros();
    degree += (zGyro + prezGyro) * float(time - preMicros) / 2000000;
    int _degree = int(degree);
    _degree += 180;
    _degree %= 360;
    if(_degree < 0){
        _degree += 180;
    }else{
        _degree -= 180;
    }

    float radian = -radians(_degree);

    /*if(radian>PI){
        radian -= 2*314*floor(_radian/(2*314))/100.0;
    }

    if(0<_radian%(2*314) && _radian%(2*314)<=314){
        radian = (float)_radian / 100;
    }
    else{
        radian = (float)(_radian - 314*2) / 100;
    }

    radian = (float)_radian / 100;*/

    preMicros = time;
    prezGyro = zGyro;
    Serial.println(radian);
    return radian;
}
//=====================================================================================//
void BMX055_Mag()
{
    unsigned int data[8];
    for (int i = 0; i < 8; i++)
    {
        Wire.beginTransmission(Addr_Mag);
        Wire.write((0x42 + i)); // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_Mag, 1); // Request 1 byte of data
        // Read 6 bytes of data
        // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data
    xMag = ((data[1] << 5) | (data[0] >> 3));
    if (xMag > 4095)
        xMag -= 8192;
    yMag = ((data[3] << 5) | (data[2] >> 3));
    if (yMag > 4095)
        yMag -= 8192;
    zMag = ((data[5] << 7) | (data[4] >> 1));
    if (zMag > 16383)
        zMag -= 32768;
}
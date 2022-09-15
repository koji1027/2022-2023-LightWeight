#include <Arduino.h>
#include <Wire.h>

#define Addr_accl 0x19
#define Addr_gyro 0x69
#define Addr_mag 0x13

class Axis
{
public:
    void init();
    void accl();
    void gyro();
    void mag();
    float convert_range(float degree);

private:
    float xaccl = 0.00;
    float yaccl = 0.00;
    float zaccl = 0.00;
    float xgyro = 0.00;
    float ygyro = 0.00;
    float zgyro = 0.00;
    int xmag = 0;
    int ymag = 0;
    int zmag = 0;
    float xgyro_offset = 0.00;
    float ygyro_offset = 0.00;
    float zgyro_offset = 0.00;
    int xmag_offset = 25;
    int ymag_offset = 25;
    float gyro_degree = 0.00;
    float gyro_radian = 0.00;
    unsigned long long pre_micro = 0;
    int pre_zgyro = 0;
    float mag_radian = 0.00;
    float mag_radian_zero = 0.00;
};

void Axis::init()
{
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_accl);
    Wire.write(0x0F); // Select PMU_Range register
    Wire.write(0x03); // Range = +/- 2g
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_accl);
    Wire.write(0x10); // Select PMU_BW register
    Wire.write(0x08); // Bandwidth = 7.81 Hz
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_accl);
    Wire.write(0x11); // Select PMU_LPW register
    Wire.write(0x00); // Normal mode, Sleep duration = 0.5ms
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_gyro);
    Wire.write(0x0F); // Select Range register
    Wire.write(0x00); // Full scale = +/- 2000 degree/s
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_gyro);
    Wire.write(0x10); // Select Bandwidth register
    Wire.write(0x07); // ODR = 100 Hz
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_gyro);
    Wire.write(0x11); // Select LPM1 register
    Wire.write(0x00); // Normal mode, Sleep duration = 2ms
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_mag);
    Wire.write(0x4B); // Select mag register
    Wire.write(0x83); // Soft reset
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_mag);
    Wire.write(0x4B); // Select mag register
    Wire.write(0x01); // Soft reset
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_mag);
    Wire.write(0x4C); // Select mag register
    Wire.write(0x00); // Normal Mode, ODR = 10 Hz
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_mag);
    Wire.write(0x4E); // Select mag register
    Wire.write(0x84); // X, Y, Z-Axis enabled
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_mag);
    Wire.write(0x51); // Select mag register
    Wire.write(0x04); // No. of Repetitions for X-Y Axis = 9
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_mag);
    Wire.write(0x52); // Select mag register
    Wire.write(0x16); // No. of Repetitions for Z-Axis = 15
    Wire.endTransmission();

    float x_offset = 0.00;
    float y_offset = 0.00;
    float z_offset = 0.00;
    for (int i = 0; i < 10; i++)
    {
        gyro();
        x_offset += xgyro;
        y_offset += ygyro;
        z_offset += zgyro;
        delay(10);
    }
    xgyro_offset = x_offset / 10.0;
    ygyro_offset = y_offset / 10.0;
    zgyro_offset = z_offset / 10.0;

    mag();
    xmag += xmag_offset;
    ymag += ymag_offset;
    mag_radian_zero = atan2(ymag, xmag);
}
//=====================================================================================//
void Axis::accl()
{
    unsigned int data[6];
    for (int i = 0; i < 6; i++)
    {
        Wire.beginTransmission(Addr_accl);
        Wire.write((2 + i)); // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_accl, 1); // Request 1 byte of data
        // Read 6 bytes of data
        // xaccl lsb, xaccl msb, yaccl lsb, yaccl msb, zaccl lsb, zaccl msb
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data to 12-bits
    xaccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
    if (xaccl > 2047)
        xaccl -= 4096;
    yaccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
    if (yaccl > 2047)
        yaccl -= 4096;
    zaccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
    if (zaccl > 2047)
        zaccl -= 4096;
    xaccl = xaccl * 0.0098; // range = +/-2g
    yaccl = yaccl * 0.0098; // range = +/-2g
    zaccl = zaccl * 0.0098; // range = +/-2g
}
//=====================================================================================//
void Axis::gyro()
{
    unsigned int data[6];
    for (int i = 0; i < 6; i++)
    {
        Wire.beginTransmission(Addr_gyro);
        Wire.write((2 + i)); // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_gyro, 1); // Request 1 byte of data
        // Read 6 bytes of data
        // xgyro lsb, xgyro msb, ygyro lsb, ygyro msb, zgyro lsb, zgyro msb
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data
    xgyro = (data[1] * 256) + data[0];
    if (xgyro > 32767)
        xgyro -= 65536;
    ygyro = (data[3] * 256) + data[2];
    if (ygyro > 32767)
        ygyro -= 65536;
    zgyro = (data[5] * 256) + data[4];
    if (zgyro > 32767)
        zgyro -= 65536;

    xgyro = xgyro * 0.06103515625; //  Full scale = +/- 125 degree/s
    ygyro = ygyro * 0.06103515625; //  Full scale = +/- 125 degree/s
    zgyro = zgyro * 0.06103515625; //  Full scale = +/- 125 degree/s
    xgyro -= xgyro_offset;
    ygyro -= ygyro_offset;
    zgyro -= zgyro_offset;

    unsigned long long micro = micros();
    gyro_degree += (micro - pre_micro) * (zgyro + pre_zgyro) / 2000000.0;
    pre_micro = micro;
    gyro_radian = convert_range(gyro_degree);
}
//=====================================================================================//
void Axis::mag()
{
    unsigned int data[8];
    for (int i = 0; i < 8; i++)
    {
        Wire.beginTransmission(Addr_mag);
        Wire.write((0x42 + i)); // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_mag, 1); // Request 1 byte of data
        // Read 6 bytes of data
        // xmag lsb, xmag msb, ymag lsb, ymag msb, zmag lsb, zmag msb
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data
    xmag = ((data[1] << 5) | (data[0] >> 3));
    if (xmag > 4095)
        xmag -= 8192;
    ymag = ((data[3] << 5) | (data[2] >> 3));
    if (ymag > 4095)
        ymag -= 8192;
    zmag = ((data[5] << 7) | (data[4] >> 1));
    if (zmag > 16383)
        zmag -= 32768;

    xmag += xmag_offset;
    ymag += ymag_offset;

    float radian = atan2(ymag, xmag);
    float mag_degree = radian * 180.0 / PI;
    mag_radian = convert_range(mag_degree) - mag_radian_zero;
}

float Axis::convert_range(float degree) //-Pi から Pi に変換
{
    int _degree = int(degree);
    _degree += 180;
    _degree %= 360;
    if (_degree < 0)
    {
        _degree += 180;
    }
    else
    {
        _degree -= 180;
    }
    float radian = _degree * PI / 180.0;
    radian = -radian;
    return radian;
}
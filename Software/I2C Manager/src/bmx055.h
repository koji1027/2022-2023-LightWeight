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
    void accl_gyro_calibration();
    void mag_calibration();
    float convert_range(float degree);
    int get_max_min(int data[], int size, bool tag);

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
    float xaccl_offset = 0.00;
    float yaccl_offset = 0.00;
    float zaccl_offset = 0.00;
    float xgyro_offset = 0.00;
    float ygyro_offset = 0.00;
    float zgyro_offset = 0.00;
    float xmag_offset = 0.00;
    float ymag_offset = 0.00;
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
    Wire.write(0x0F); // Bandwidth = 7.81 Hz
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
    Wire.write(0x01); // ODR = 100 Hz
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
    Wire.write(0x07); // Normal Mode, ODR = 10 Hz
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

void Axis::accl_gyro_calibration()
{
    int i = 0;
    int buff_size = 1000;
    float buff_xaccl, buff_yaccl, buff_zaccl = 0.00;
    int buff_xgyro, buff_ygyro, buff_zgyro = 0;
    while (i < buff_size + 101)
    {
        if (i < 100)
        {
            accl();
            gyro();
            delay(2);
        }
        else if (i < buff_size + 100)
        {
            accl();
            gyro();
            buff_xaccl += xaccl;
            buff_yaccl += yaccl;
            buff_zaccl += zaccl;
            buff_xgyro += xgyro;
            buff_ygyro += ygyro;
            buff_zgyro += zgyro;
            delay(2);
        }
        else if (i == buff_size + 100)
        {
            xaccl_offset = float(buff_xaccl) / float(buff_size);
            yaccl_offset = float(buff_yaccl) / float(buff_size);
            zaccl_offset = float(buff_zaccl) / float(buff_size);
            xgyro_offset = float(buff_xgyro) / float(buff_size);
            ygyro_offset = float(buff_ygyro) / float(buff_size);
            zgyro_offset = float(buff_zgyro) / float(buff_size);
        }
    }
}

void Axis::mag_calibration()
{
    int i = 0;
    int buff_size = 1000;
    int buff_xmag[buff_size], buff_ymag[buff_size];
    while (i < buff_size + 101)
    {
        if (i < 100)
        {
            mag();
            delay(2);
        }
        else if (i < buff_size + 100)
        {
            mag();
            buff_xmag[i - 100] = xmag;
            buff_ymag[i - 100] = ymag;
            delay(2);
        }
        else if (i == buff_size + 100)
        {
            int xmag_max, xmag_min, ymag_max, ymag_min;
            xmag_max = get_max_min(buff_xmag, buff_size, true);
            xmag_min = get_max_min(buff_xmag, buff_size, false);
            ymag_max = get_max_min(buff_ymag, buff_size, true);
            ymag_min = get_max_min(buff_ymag, buff_size, false);
            xmag_offset = float(xmag_max + xmag_min) / 2.0;
            ymag_offset = float(ymag_max + ymag_min) / 2.0;
        }
    }
}

int Axis::get_max_min(int data[], int size, bool tag)
{
    if (tag)
    {
        int max = data[0];
        for (int i = 1; i < size; i++)
        {
            if (max < data[i])
            {
                max = data[i];
            }
        }
        return max;
    }
    else
    {
        int min = data[0];
        for (int i = 1; i < size; i++)
        {
            if (min > data[i])
            {
                min = data[i];
            }
        }
        return min;
    }
}
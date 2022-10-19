#include <Arduino.h>
#include <Wire.h>

#define Addr_accl 0x19
#define Addr_gyro 0x69
#define Addr_mag 0x13

class Axis
{
public:
    void init();
    void calibration();
    void gyro_reset();
    void gyro();
    void cal_angle();
    void send();
    void show(bool accl, bool gyro, bool mag);
    // void cal_angle_mag();
    // void mag();
    // void accl();
    // void adjust_angle();

private:
    float xgyro = 0.00;
    float ygyro = 0.00;
    float zgyro = 0.00;
    float xgyro_offset = 0.00;
    float ygyro_offset = 0.00;
    float zgyro_offset = 0.00;
    unsigned long long pre_time = 0;
    float pre_zgyro = 0.00;
    float gyro_degree = 0.00;
    // float xaccl = 0.00;
    // float yaccl = 0.00;
    // float zaccl = 0.00;
    // int xmag = 0;
    // int ymag = 0;
    // int zmag = 0;
    // float xaccl_offset = 0.00;
    // float yaccl_offset = 0.00;
    // float zaccl_offset = 0.00;
    // int xmag_offset = 0;
    // int ymag_offset = 0;
    // int zmag_offset = 0;
    // float mag_radian = 0.00;
    // float mag_radian_offset = 0.00;
};

void Axis::init()
{

    Wire.beginTransmission(Addr_accl);
    Wire.write(0x0F); // Select PMU_Range register
    Wire.write(0x03); // Range = +/- 2g
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(Addr_accl);
    Wire.write(0x10); // Select PMU_BW register
    Wire.write(0x08); // Bandwidth = 7.81 Hz
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(Addr_accl);
    Wire.write(0x11); // Select PMU_LPW register
    Wire.write(0x00); // Normal mode, Sleep duration = 0.5ms
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(Addr_gyro);
    Wire.write(0x0F); // Select Range register
    Wire.write(0x00); // Full scale = +/- 500 degree/s
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(Addr_gyro);
    Wire.write(0x10); // Select Bandwidth register
    Wire.write(0x01); // ODR = 100 Hz
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(Addr_gyro);
    Wire.write(0x11); // Select LPM1 register
    Wire.write(0x00); // Normal mode, Sleep duration = 2ms
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(Addr_mag);
    Wire.write(0x4B); // Select Mag register
    Wire.write(0x83); // Soft reset
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(Addr_mag);
    Wire.write(0x4B); // Select Mag register
    Wire.write(0x01); // Soft reset
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(Addr_mag);
    Wire.write(0x4C); // Select Mag register
    Wire.write(0x00); // Normal Mode, ODR = 10 Hz
    Wire.endTransmission();

    Wire.beginTransmission(Addr_mag);
    Wire.write(0x4E); // Select Mag register
    Wire.write(0x84); // X, Y, Z-Axis enabled
    Wire.endTransmission();

    Wire.beginTransmission(Addr_mag);
    Wire.write(0x51); // Select Mag register
    Wire.write(0x04); // No. of Repetitions for X-Y Axis = 9
    Wire.endTransmission();

    Wire.beginTransmission(Addr_mag);
    Wire.write(0x52); // Select Mag register
    Wire.write(0x16); // No. of Repetitions for Z-Axis = 15
    Wire.endTransmission();

    calibration();

    delay(500);
}

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

    xgyro = xgyro * 0.0152587890625*4; //  Full scale = +/- 500 degree/s
    ygyro = ygyro * 0.0152587890625*4; //  Full scale = +/- 500 degree/s
    zgyro = zgyro * 0.0152587890625*4; //  Full scale = +/- 500 degree/s

    xgyro += xgyro_offset;
    ygyro += ygyro_offset;
    zgyro += zgyro_offset;
}

void Axis::show(bool accl, bool gyro, bool mag)
{
    if (accl)
    {
        /*Serial.print("xaccl = ");
        Serial.print(xaccl);
        Serial.print(" yaccl = ");
        Serial.print(yaccl);
        Serial.print(" zaccl = ");
        Serial.println(zaccl);*/
    }
    if (gyro)
    {
        /*Serial.print("xgyro = ");
        Serial.print(xgyro);
        Serial.print(" ygyro = ");
        Serial.print(ygyro);
        Serial.print(" zgyro = ");
        Serial.print(zgyro);
        Serial.print(" gyro_degree = ");
        Serial.print(gyro_degree);
        Serial.println(" °");*/
        Serial.println(zgyro);
    }
    if (mag)
    {
        /* Serial.print("xmag = ");
        Serial.print(xmag);
        Serial.print(",");
        Serial.println(ymag);
        // Serial.print(" zmag = ");
        // Serial.println(zmag);
        /*Serial.print(xmag);
        Serial.print(",");
        Serial.print(ymag);
        Serial.println(",");*/
    }
    if (!accl && !gyro && !mag)
    {
        Serial.print("gyro_degree = ");
        Serial.println(gyro_degree);
        /*Serial.print(" mag_degree = ");
        Serial.print(degrees(mag_radian));
        Serial.print(" integrated_degree = ");
        Serial.println(integrated_degree);*/
    }
}

void Axis::cal_angle()
{
    gyro();
    unsigned long long now = micros();
    float dt = (now - pre_time) / 1000000.0;
    pre_time = now;
    float dtheta = (zgyro + pre_zgyro) / 2.0 * dt;
    pre_zgyro = zgyro;
    gyro_degree += dtheta;
    if (gyro_degree > 180)
        gyro_degree -= 360;
    else if (gyro_degree <= -180)
        gyro_degree += 360;
    Serial.println(gyro_degree);
}

void Axis::gyro_reset()
{
    gyro_degree = 0.00;
}

void Axis::send()
{
    float rad = radians(gyro_degree);
    rad = rad / PI;
    rad += 1;
    int data = rad * 100;
    //Serial1.write(255);
    Serial1.write(data);
    Serial.println(data);
}

void Axis::calibration()
{
    Serial.println("Calibrating...");
    Serial.println("Do not move the sensor");
    float sum_xgyro = 0.00;
    float sum_ygyro = 0.00;
    float sum_zgyro = 0.00;
    for (int i = 0; i < 351; i++)
    {
        if (i < 50)
        {
            gyro();
            delay(5);
        }
        else if (i < 350)
        {
            gyro();
            sum_xgyro -= xgyro;
            sum_ygyro -= ygyro;
            sum_zgyro -= zgyro;
            delay(5);
        }
        else
        {
            xgyro_offset = sum_xgyro / 300.0;
            ygyro_offset = sum_ygyro / 300.0;
            zgyro_offset = sum_zgyro / 300.0;
        }
    }

    /*Serial.println("Rotate the sensor 360 degrees");
    int xmag_max, xmag_min, ymag_max, ymag_min;
    for (int i = 0; i < 551; i++)
    {
        if (i < 50)
        {
            mag();
            delay(5);
        }
        else if (i == 50)
        {
            xmag_max = xmag;
            xmag_min = xmag;
            ymag_max = ymag;
            ymag_min = ymag;
        }
        else if (i < 550)
        {
            mag();
            Serial.print("xmag: ");
            Serial.print(xmag);
            Serial.print(" ymag: ");
            Serial.println(ymag);
            if (xmag > xmag_max)
                xmag_max = xmag;
            if (xmag < xmag_min)
                xmag_min = xmag;
            if (ymag > ymag_max)
                ymag_max = ymag;
            if (ymag < ymag_min)
                ymag_min = ymag;
            delay(5);
        }
        else
        {
            xmag_offset = -float(xmag_max + xmag_min) / 2.0;
            ymag_offset = -float(ymag_max + ymag_min) / 2.0;
        }
    }
    Serial.println("Calibration complete");
    Serial.print("Calibration Result: ");
    Serial.print("xmag_offset = ");
    Serial.print(xmag_offset);
    Serial.print(", ymag_offset = ");
    Serial.print(ymag_offset);
    Serial.print(", zgyro_offset = ");
    Serial.println(zgyro_offset);*/
}

/*
void Axis::accl()
{
    unsigned int data[6];
    for (int i = 0; i < 6; i++)
    {
        Wire2.beginTransmission(Addr_accl);
        Wire2.write((2 + i)); // Select data register
        Wire2.endTransmission();
        Wire2.requestFrom(Addr_accl, 1); // Request 1 byte of data
        // Read 6 bytes of data
        // xaccl lsb, xaccl msb, yaccl lsb, yaccl msb, zaccl lsb, zaccl msb
        if (Wire2.available() == 1)
            data[i] = Wire2.read();
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

    xaccl += xaccl_offset;
    yaccl += yaccl_offset;
    zaccl += zaccl_offset;
}

void Axis::mag()
{
    unsigned int data[8];
    for (int i = 0; i < 8; i++)
    {
        unsigned int data[8];
        for (int i = 0; i < 8; i++)
        {
            Wire2.beginTransmission(Addr_mag);
            Wire2.write((0x42 + i)); // Select data register
            Wire2.endTransmission();
            Wire2.requestFrom(Addr_mag, 1); // Request 1 byte of data
            // Read 6 bytes of data
            // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
            if (Wire2.available() == 1)
                data[i] = Wire2.read();
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
    }

    xmag += xmag_offset;
    ymag += ymag_offset;
}

void Axis::cal_angle_mag()
{
    mag();
    for (int i = MAG_LPF_BUFF - 1; i > 0; i--)
    {
        mag_rad_buff[i] = mag_rad_buff[i - 1];
    }
    mag_rad_buff[0] = (atan2(ymag, xmag) - mag_radian_offset);
    if (mag_rad_buff[0] > PI)
    {
        mag_rad_buff[0] -= 2 * PI;
    }
    else if (mag_rad_buff[0] < -PI)
    {
        mag_rad_buff[0] += 2 * PI;
    }
    float sum = 0;
    for (int i = 0; i < MAG_LPF_BUFF; i++)
    {
        sum += mag_rad_buff[i];
    }
    mag_radian = sum / float(MAG_LPF_BUFF);
}

void Axis::adjust_angle()
{
    float diff = abs(gyro_degree - degrees(mag_radian));
    if (diff > 4)
    {
        gyro_degree = degrees(mag_radian);
    }
    integrated_degree = (gyro_degree + degrees(mag_radian)) / 2.0;
}

*/
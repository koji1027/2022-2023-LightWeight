#include <Arduino.h>
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

#include <Wire.h>
// BMX055 加速度センサのI2Cアドレス
#define Addr_Accl 0x19 // (JP1,JP2,JP3 = Openの時)
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69 // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13 // (JP1,JP2,JP3 = Openの時)

class BMX055
{
public:
  void Init();
  void Accl();
  void Gyro();
  void Mag();
  void Show();
  void Gyro_calib();
  void Gyro_rad();
  void Mag_calib();
  void Mag_set_zero();
  void Mag_rad();
  void Set_zero();
  float GyroRad = 0.00;
  float MagRad = 0.00;

private:
  float xAccl = 0.00;
  float yAccl = 0.00;
  float zAccl = 0.00;
  float xGyro = 0.00;
  float yGyro = 0.00;
  float zGyro = 0.00;
  int xMag = 0;
  int yMag = 0;
  int zMag = 0;
  float zGyroOffset = 0.00;
  unsigned long preTime = 0;
  float prezGyro = 0.00;
  float GyroDeg = 0.00;
  float xMagOffset = 0;
  float yMagOffset = 0;
  float MagRadOffset = 0.00;
};

BMX055 bmx055;

void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  Serial.begin(115200);
  // BMX055 初期化
  bmx055.Init();
  delay(2000);
  bmx055.Gyro_calib();
  bmx055.Mag_calib();
  bmx055.Set_zero();
}

void loop()
{
  bmx055.Accl();
  bmx055.Gyro();
  bmx055.Mag();
  bmx055.Gyro_rad();
  bmx055.Mag_rad();
  delay(10);
}

void BMX055::Init()
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
  Wire.write(0x00); // Full scale = +/- 125 degree/s
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
}

void BMX055::Accl()
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

void BMX055::Gyro()
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

  xGyro = xGyro * 0.0610360875867857; //  Full scale = +/- 125 degree/s
  yGyro = yGyro * 0.0610360875867857; //  Full scale = +/- 125 degree/s
  zGyro = zGyro * 0.0610360875867857; //  Full scale = +/- 125 degree/s

  zGyro += zGyroOffset;
}

void BMX055::Mag()
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
  xMag += xMagOffset;
  yMag += yMagOffset;
}

void BMX055::Show()
{
  Serial.println("--------------------------------------");

  Serial.print("Accl= ");
  Serial.print(xAccl);
  Serial.print(",");
  Serial.print(yAccl);
  Serial.print(",");
  Serial.print(zAccl);
  Serial.println("");

  Serial.print("Gyro= ");
  Serial.print(xGyro);
  Serial.print(",");
  Serial.print(yGyro);
  Serial.print(",");
  Serial.print(zGyro);
  Serial.println("");

  Serial.print("Mag= ");
  Serial.print(xMag);
  Serial.print(",");
  Serial.print(yMag);
  Serial.print(",");
  Serial.print(zMag);
  Serial.println("");

  Serial.print("Gyro_Offset= ");
  Serial.println(zGyroOffset);

  Serial.print("Mag_Offset= ");
  Serial.print(xMagOffset);
  Serial.print(",");
  Serial.println(yMagOffset);
}

void BMX055::Gyro_calib()
{
  Serial.println("Gyro calibration start");
  Serial.println("Don't move the sensor");
  int buff_size = 1000;
  float zGyro_sum = 0.00;
  for (int i = 0; i < buff_size + 101; i++)
  {
    if (i < 100)
    {
      Gyro();
      delay(3);
    }
    else if (i < buff_size + 100)
    {
      Gyro();
      zGyro_sum -= zGyro;
      delay(3);
    }
    else
    {
      zGyroOffset = float(zGyro_sum) / float(buff_size);
    }
  }
  Serial.println("Gyro calibration end");
}

void BMX055::Gyro_rad()
{
  Gyro();
  unsigned long now = micros();
  float dt = (now - preTime) / 1000000.0;
  preTime = now;
  GyroDeg += (zGyro + prezGyro) * dt / 2.0;
  prezGyro = zGyro;
  int _GyroDeg = int(GyroDeg) * -1;
  _GyroDeg += 180;
  _GyroDeg %= 360;
  _GyroDeg -= 180;
  if (_GyroDeg < -180)
  {
    _GyroDeg += 360;
  }
  GyroRad = _GyroDeg * PI / 180.0;
}

void BMX055::Mag_calib()
{
  Serial.println("Mag calibration start");
  Serial.println("Rotate the sensor in all directions");
  int buff_size = 1000;
  int xMag_buff[buff_size];
  int yMag_buff[buff_size];
  for (int i = 0; i < buff_size + 101; i++)
  {
    if (i < 100)
    {
      Mag();
      delay(3);
    }
    else if (i < buff_size + 100)
    {
      Mag();
      xMag_buff[i - 100] = xMag;
      yMag_buff[i - 100] = yMag;
      delay(3);
    }
    else
    {
      int xMag_max = xMag_buff[0];
      int xMag_min = xMag_buff[0];
      int yMag_max = yMag_buff[0];
      int yMag_min = yMag_buff[0];
      for (int i = 0; i < buff_size - 1; i++)
      {
        if (xMag_buff[i] > xMag_max)
        {
          xMag_max = xMag_buff[i];
        }
        if (xMag_buff[i] < xMag_min)
        {
          xMag_min = xMag_buff[i];
        }
        if (yMag_buff[i] > yMag_max)
        {
          yMag_max = yMag_buff[i];
        }
        if (yMag_buff[i] < yMag_min)
        {
          yMag_min = yMag_buff[i];
        }
      }
      xMagOffset = -(float(xMag_max + xMag_min) / 2.0);
      yMagOffset = -(float(yMag_max + yMag_min) / 2.0);
    }
    Mag();
  }
  Serial.println("Mag calibration end");
}

void BMX055::Mag_rad()
{
  float radians = atan2(yMag, xMag);
  MagRad = radians;
  MagRad += MagRadOffset;
  if (MagRad > PI)
  {
    MagRad -= 2 * PI;
  }
  else if (MagRad < -PI)
  {
    MagRad += 2 * PI;
  }
}

void BMX055::Set_zero()
{
  int buff_size = 100;
  float MagRad_sum = 0.00;
  for (int i = 0; i < buff_size; i++)
  {
    Mag();
    Mag_rad();
    MagRad_sum += MagRad;
    delay(3);
  }
  MagRadOffset = -MagRad_sum / float(buff_size);
  GyroDeg = 0.0;
  GyroRad = 0.0;
}
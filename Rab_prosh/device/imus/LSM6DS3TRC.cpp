#pragma once
#include "device/imus/LSM6DS3TRC.h"

static BiquadFilter FiltAcc[3];

static BiquadFilter FiltGyr[3];

static IMU_Proc IMU_DoneProc;

static long GyroShift[3]{ 0, 0, 0 };

struct IMU_DataBufer {short temp; short gx; short gy; short gz; short ax; short ay; short az; };

static inline void IMU_Calibrate(IMU_DataBufer& imu, long GyrLim, long AccZero, long AccMax)
{
  long len = imu.gx * imu.gx + imu.gy * imu.gy + imu.gz * imu.gz;
  if (len > (GyrLim * GyrLim)) return;

  static float x = imu.gx, y = imu.gy, z = imu.gz;

  const float alpha = 0.001f;

  x = x * (1.0f - alpha) + alpha * imu.gx;
  y = y * (1.0f - alpha) + alpha * imu.gy;
  z = z * (1.0f - alpha) + alpha * imu.gz;

  GyroShift[0] = x;
  GyroShift[1] = y;
  GyroShift[2] = z;

  len = imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az;
  if (abs(len - AccZero * AccZero) > (AccMax * AccMax)) return;

  len = sqrtf(len);

  static float acc = len;

  acc = acc * (1.0f - alpha) + alpha * len;
}

static void IMU_CallbackProc(unsigned char Address, const unsigned char* Data, unsigned char Size)
{
  IMU_DataBufer& data = *(IMU_DataBufer*)Data;

  data.ax = Rev16(data.ax);
  data.ay = Rev16(data.ay);
  data.az = Rev16(data.az);

  data.gx = Rev16(data.gx);
  data.gy = Rev16(data.gy);
  data.gz = Rev16(data.gz);

  data.ax = biquad_apply(&FiltAcc[0], data.ax);
  data.ay = biquad_apply(&FiltAcc[1], data.ay);
  data.az = biquad_apply(&FiltAcc[2], data.az);

  data.gx = biquad_apply(&FiltGyr[0], data.gx);
  data.gy = biquad_apply(&FiltGyr[1], data.gy);
  data.gz = biquad_apply(&FiltGyr[2], data.gz);

  static IMU_DataBufer test = *(IMU_DataBufer*)Data;
  test = *(IMU_DataBufer*)Data;

  if (CalibDataIMU.AllowedCalib) IMU_Calibrate(data, 50, 4096, 500);

  static IMU_Data result;

  static float alpha = 0.01;

  result.RawAcc.X = (result.RawAcc.X * (1.0f - alpha)) + test.ax * alpha;
  result.RawAcc.Y = (result.RawAcc.Y * (1.0f - alpha)) + test.ay * alpha;
  result.RawAcc.Z = (result.RawAcc.Z * (1.0f - alpha)) + test.az * alpha;

  result.RawGyr.X = (result.RawGyr.X * (1.0f - alpha)) + test.gx * alpha;
  result.RawGyr.Y = (result.RawGyr.Y * (1.0f - alpha)) + test.gy * alpha;
  result.RawGyr.Z = (result.RawGyr.Z * (1.0f - alpha)) + test.gz * alpha;

  result.Temp = 21 + (((long)Rev16(data.temp)) * 128) / 42735;

  result.Acc.X = Normalize(data.ax, 1.0f, CalibDataIMU.Data.Acc.X[0], CalibDataIMU.Data.Acc.X[1]);
  result.Acc.Y = Normalize(data.ay, 1.0f, CalibDataIMU.Data.Acc.Y[0], CalibDataIMU.Data.Acc.Y[1]);
  result.Acc.Z = Normalize(data.az, 1.0f, CalibDataIMU.Data.Acc.Z[0], CalibDataIMU.Data.Acc.Z[1]);

  result.Gyr.X = ((float)(data.gx - GyroShift[0])) / 16.384f;
  result.Gyr.Y = ((float)(data.gy - GyroShift[1])) / 16.384f;
  result.Gyr.Z = ((float)(data.gz - GyroShift[2])) / 16.384f;

  IMU_DoneProc(result);
}

static unsigned char IMU_Buffer[32];
static I2C_Request IMU_Device = { &IMU_CallbackProc, IMU_Buffer, sizeof(IMU_Buffer), 0 };

LSM6DS3TRC::LSM6DS3TRC()
{

}

LSM6DS3TRC::~LSM6DS3TRC()
{

}

void LSM6DS3TRC::Init()
{
  lpf2p_init(&FiltAcc[0], 500.0f, 25.0f);
  lpf2p_init(&FiltAcc[1], 500.0f, 25.0f);
  lpf2p_init(&FiltAcc[2], 500.0f, 25.0f);
  
  lpf2p_init(&FiltGyr[0], 500.0f, 25.0f);
  lpf2p_init(&FiltGyr[1], 500.0f, 25.0f);
  lpf2p_init(&FiltGyr[2], 500.0f, 25.0f);
  
  for (int a = 0; a < 100000; a++) { asm volatile("NOP"); }

  unsigned char wai = IMU_GetReg(IMU_Addr, 0x0F);

  IMU_SetReg(IMU_Addr, 0x10, 0x8C); // FS_XL_10 : ±8g | ODR_XL_1010 : : 1.66kHz

  IMU_SetReg(IMU_Addr, 0x11, 0x8C); // FS_G_10 : 2000dps | ODR_G_1010 : 1.66kHz

  IMU_SetReg(IMU_Addr, 0x12, 0x46); // BLE | IF_INC | BDU

  IMU_SetReg(IMU_Addr, 0x13, 0x02); // LPF1_SEL_G

  IMU_SetReg(IMU_Addr, 0x15, 0x02); // FTYPE_10  173Hz

  IMU_SetReg(IMU_Addr, 0x17, 0x80); // LPF2_XL_EN

  for (int a = 0; a < 100000; a++) { asm volatile("NOP"); }
}

void LSM6DS3TRC::IMU_SetBank(unsigned char Bank)
{
  unsigned char reg[2];
  reg[0] = 0x7F; reg[1] = Bank << 4; // REG_BANK_SEL   -    USER_BANK
  I2C1_Write(IMU_Addr, reg, 2);
  I2C1_Stop();
}

void LSM6DS3TRC::Get(IMU_Data& Data)
{
  struct { short ax; short ay; short az; short gx; short gy; short gz; short temp; short mx; short my; short mz; } data;

  I2C1_Write(IMU_Addr, IMU_Read_Reg);
  I2C1_Read(IMU_Addr, &data, sizeof(data));
  I2C1_Stop();

  Data.Temp = 21 + (((long)Rev16(data.temp)) * 128) / 42735;

  Data.Acc.X = Rev16(data.ax);
  Data.Acc.Y = Rev16(data.ay);
  Data.Acc.Z = Rev16(data.az);

  Data.Gyr.X = Rev16(data.gx);
  Data.Gyr.Y = Rev16(data.gy);
  Data.Gyr.Z = Rev16(data.gz);
}

void LSM6DS3TRC::GetAsync(const IMU_Proc& DoneProc)
{
  IMU_DoneProc = DoneProc;
  I2C1_Trans(&IMU_Device, IMU_Addr, &IMU_Read_Reg, 1, sizeof(IMU_DataBufer));
}
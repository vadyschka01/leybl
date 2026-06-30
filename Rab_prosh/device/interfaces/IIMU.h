#pragma once
#include "i2c.h"
#include "Biquad.h"
#include "extend/MathFunc.h"

#pragma pack(push,1)
struct IMU_Data
{
  long Temp;
  struct { float X, Y, Z; } Acc;
  struct { float X, Y, Z; } RawAcc;
  struct { float X, Y, Z; } Gyr;
  struct { float X, Y, Z; } RawGyr;
  struct { float X, Y, Z; } Mag;
  struct { float X, Y, Z; } RawMag;
};

struct XYZ_Calib
{
  short X[2] = { 0,0 };
  short Y[2] = { 0,0 };
  short Z[2] = { 0,0 };
};

struct XYZ_IMU_DATA
{
  char Writed = sizeof(XYZ_IMU_DATA);
  XYZ_Calib Acc;
  //XYZ_Calib Mag;
};

struct IMU_Calib_Data
{
  bool AllowedCalib = false;
  bool CalibAccNeed = true;
  bool CalibMagNeed = true;
  XYZ_IMU_DATA Data;
};

struct IMU_Info
{
  unsigned char Addrs[3] = { 0x68, 0x6A, 0x68 }; // Адреса устройств на шине i2c
  unsigned char WhoIAmReg[3] = { 0x00, 0x0F, 0x75 }; // Регистры Who i am
  unsigned char ExpectedIDs[3] = { 0xEA, 0x6A, 0x3B }; // Ожидаемые значения
};
#pragma pack(pop)

typedef void (*IMU_Proc)(IMU_Data&);

extern IMU_Calib_Data CalibDataIMU;

void IMU_SetReg(unsigned char Addr, unsigned char Reg, unsigned char Value);

unsigned char IMU_GetReg(unsigned char Addr, unsigned char Reg);

class IIMU
{
protected:
  virtual ~IIMU() = default;
public:
  virtual void Init() = 0;

  virtual void Get(IMU_Data& Data) = 0;

  virtual void GetAsync(const IMU_Proc& DoneProc) = 0;
};

IIMU* TryFindIMU(bool& find);
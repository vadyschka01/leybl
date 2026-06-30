#pragma once
#include "i2c.h"
#include "extend/MathFunc.h"

struct MAG_Data
{
  float X, Y, Z;
  float RawX, RawY, RawZ;
};

struct MAG_Calib_Data
{
  bool CalibNeed = true;
  short X[2] = { 0,0 };
  short Y[2] = { 0,0 };
  short Z[2] = { 0,0 };
};

extern MAG_Calib_Data ExternMagCalibData;

struct Mag_Info
{
  unsigned char Addrs[1] = { 0x0E }; // Адреса устройств на шине i2c
  unsigned char WhoIAmReg[1] = { 0x00 }; // Регистры Who i am
  unsigned char ExpectedIDs[1] = { 0x10 }; // Ожидаемые значения
};

void MAG_SetReg(unsigned char Addr, unsigned char Reg, unsigned char Value);

class IMag
{
protected:
  virtual ~IMag() = default;

public:

  virtual void Init() = 0;

  virtual bool Get(MAG_Data& Data) = 0;

  virtual void GetAsync(void (*DoneProc)(MAG_Data& Data)) = 0;
};

IMag* TryFindMag(bool& find);
#pragma once
#include <i2c.h>

struct BarometersInfo
{
  unsigned char Addrs[2] = { 0x76, 0x5C }; // Адреса устройств на шине i2c
  unsigned char WhoIAmReg[2] = { 0xD0, 0x0F }; // Регистры Who i am
  unsigned char ExpectedIDs[2] = { 0x58, 0xB3 }; // Ожидаемые значения
};

struct BAR_Data
{
  long Temp;
  float Pressure;
};
inline void (*BAR_DoneProc)(bool Ready, BAR_Data& Data);
void BAR_SetReg(unsigned char Addr, unsigned char Reg, unsigned char Value);

class IBar
{
protected:
  virtual ~IBar() = default;

public:

  virtual void Init() = 0;

  virtual float GetAltitude(float zero, float press, float temp) = 0;

  virtual void GetAsync(void (*DoneProc)(bool Ready, BAR_Data& Data)) = 0;

  virtual long GetData(long* Temp) = 0;
};

IBar* TryFindBar(bool& find);
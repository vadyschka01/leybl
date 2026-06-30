#pragma once
#include "i2c.h"
#include "MathFunc.h"

struct EPPInfo
{
  unsigned char Addrs[1] = { 0x50 }; // Адреса устройств на шине i2c
};

struct EEP_Data
{
  static const unsigned short Count = 64;

  unsigned short Addr;
  unsigned short Size;
  unsigned char Data[Count];
};

class IEEPROM
{
protected:
  virtual ~IEEPROM() = default;

public:

  virtual void Init() = 0;

  virtual void GetAsunc(unsigned short Address, unsigned short Size, void (*DoneProc)(bool Ready, EEP_Data& Data)) = 0;

  virtual void SetAsunc(unsigned short Address, const void* Data, unsigned short Size, void (*DoneProc)(bool Ready, EEP_Data& Data)) = 0;

  virtual void Read(unsigned short Addr, void* Data, unsigned short Size) = 0;

  virtual void Write(unsigned short Addr, const void* Data, unsigned short Size) = 0;
};

IEEPROM* TryFindEEPROM(bool& find);
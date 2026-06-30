#pragma once
#include "device/interfaces/IEEPROM.h"

class AT24C256 : IEEPROM
{
public:
  AT24C256();

  ~AT24C256();

  unsigned char EEP_Addr = 0x50;

  void Init();

  void GetAsunc(unsigned short Address, unsigned short Size, void (*DoneProc)(bool Ready, EEP_Data& Data));

  void SetAsunc(unsigned short Address, const void* Data, unsigned short Size, void (*DoneProc)(bool Ready, EEP_Data& Data));

  void Read(unsigned short Addr, void* Data, unsigned short Size);

  void Write(unsigned short Addr, const void* Data, unsigned short Size);
};
#pragma once
#include "device/interfaces/IMag.h"

class IST8310 : IMag
{
public:
  IST8310();

  ~IST8310();

  unsigned char Mag_Addr = 0x0E;

  unsigned char Mag_Read_Reg = 2U;

  void Init();

  bool Get(MAG_Data& Data);

  void GetAsync(void (*DoneProc)(MAG_Data& Data));
};
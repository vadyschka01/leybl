#pragma once
#include "device/interfaces/IMag.h"
#include "device/mags/IST8310.h"

Mag_Info MagInfo;
MAG_Calib_Data ExternMagCalibData;

void MAG_SetReg(unsigned char Addr, unsigned char Reg, unsigned char Value)
{
  unsigned char reg[2];
  reg[0] = Reg; reg[1] = Value;
  I2C2_Write(Addr, reg, 2);
  I2C2_Stop();
}

IMag* TryFindMag(bool& find)
{
  I2C2_Init();
  IMag* Mag = nullptr;
  for (int i = 0; i < sizeof(MagInfo.Addrs); i++)
  {
    find = I2C2_CheckDeviceWhoAmI(MagInfo.Addrs[i], MagInfo.WhoIAmReg[i], MagInfo.ExpectedIDs[i]);
    if (find && i == 0)
    {
      static IST8310 magIST8310;
      Mag = (IMag*)&magIST8310;
      break;
    }
    for (int a = 0; a < 10000000; a++) { asm volatile("NOP"); }
  }
  if (find) Mag->Init();
  return Mag;
}
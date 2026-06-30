#pragma once
#include <device/interfaces/IBar.h>
#include <device/bars/BMP280.h>
#include <device/bars/LPS22HH.h>

BarometersInfo BarsInfo;

void BAR_SetReg(unsigned char Addr, unsigned char Reg, unsigned char Value)
{
  unsigned char reg[2];
  reg[0] = Reg; reg[1] = Value;
  I2C1_Write(Addr, reg, 2);
  I2C1_Stop();
}

IBar* TryFindBar(bool& find)
{
  I2C1_Init();
  IBar* Bar = nullptr;
  for (int i = 0; i < sizeof(BarsInfo.Addrs); i++)
  {
    find = I2C1_CheckDeviceWhoAmI(BarsInfo.Addrs[i], BarsInfo.WhoIAmReg[i], BarsInfo.ExpectedIDs[i]);
    if (find && i == 0)
    {
      static BMP280 barBMP280;
      Bar = (IBar*)&barBMP280;
      break;
    }
    else if (find && i == 1)
    {
      static LPS22HH barLPS22HH;
      Bar = (IBar*)&barLPS22HH;
      break;
    }
    for (int a = 0; a < 10000000; a++) { asm volatile("NOP"); }
  }
  if (find) Bar->Init();
  return Bar;
}
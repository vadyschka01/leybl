#pragma once
#include "device/interfaces/IEEPROM.h"
#include "device/eeproms/AT24C256.h"

EPPInfo EPP_Info;

IEEPROM* TryFindEEPROM(bool& find)
{
  I2C1_Init();
  IEEPROM* epp = nullptr;
  for (int i = 0; i < sizeof(EPP_Info.Addrs); i++)
  {
    find = I2C1_CheckDevice(EPP_Info.Addrs[i]);
    if (find && i == 0)
    {
      static AT24C256 eepAT24C256;
      epp = (IEEPROM*)&eepAT24C256;
      break;
    }
    for (int a = 0; a < 10000000; a++) { asm volatile("NOP"); }
  }
  if (find) epp->Init();
  return epp;
}
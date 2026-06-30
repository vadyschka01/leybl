#pragma once
#include <string.h>
#include "device/eeproms/AT24C256.h"

void (*EEP_DoneProc)(bool Ready, EEP_Data& Data);

static void EEP_CallbackProc(unsigned char Address, const unsigned char* Data, unsigned char Size)
{
  EEP_Data result;
  result.Addr = Address;
  result.Size = Size;
  if (Size) memcpy(result.Data, Data, Size);

  EEP_DoneProc(Size, result);
}

static unsigned char EEP_Buffer[EEP_Data::Count + 2];
static I2C_Request EEP_Device = { &EEP_CallbackProc, EEP_Buffer, sizeof(EEP_Buffer), 0 };

AT24C256::AT24C256()
{

}

AT24C256::~AT24C256()
{

}

void AT24C256::Init()
{
  return; //” этой нет никакой особой инициализации
}

void AT24C256::GetAsunc(unsigned short Address, unsigned short Size, void (*DoneProc)(bool Ready, EEP_Data& Data))
{
  if (Size >= EEP_Data::Count) return;

  EEP_DoneProc = DoneProc;
  Address = Rev16(Address);

  I2C1_Trans(&EEP_Device, EEP_Addr, &Address, 2, Size);
}

void AT24C256::SetAsunc(unsigned short Address, const void* Data, unsigned short Size, void (*DoneProc)(bool Ready, EEP_Data& Data))
{
  if (Size >= EEP_Data::Count) return;

  EEP_DoneProc = DoneProc;

  unsigned char buf[EEP_Data::Count + 2];

  buf[0] = Address & 0xFF;
  buf[1] = (Address >> 8) & 0xFF;

  memcpy(buf + 2, Data, Size);

  I2C1_Trans(&EEP_Device, EEP_Addr, buf, Size + 2, 0);
}

void AT24C256::Read(unsigned short Addr, void* Data, unsigned short Size)
{
  Addr = Rev16(Addr);
  I2C1_Write(EEP_Addr, &Addr, 2);
  I2C1_Read(EEP_Addr, Data, Size);
  I2C1_Stop();
}

void AT24C256::Write(unsigned short Addr, const void* Data, unsigned short Size)
{
  Addr = Rev16(Addr);
  I2C1_Write2(EEP_Addr, &Addr, 2, Data, Size);
  I2C1_Stop();
}
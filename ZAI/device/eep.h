#pragma once

struct EEP_Data
{
  static const unsigned short Count = 64;
  
  unsigned short Addr;
  unsigned short Size;
  unsigned char Data[Count];
};

bool EEP_Init();

void EEP_GetAsunc(unsigned short Address, unsigned short Size, void (*DoneProc)(bool Ready, EEP_Data& Data));
void EEP_SetAsunc(unsigned short Address, const void* Data, unsigned short Size, void (*DoneProc)(bool Ready, EEP_Data& Data));

void EEP_Read(unsigned short Addr, void* Data, unsigned short Size);
void EEP_Write(unsigned short Addr, const void* Data, unsigned short Size);

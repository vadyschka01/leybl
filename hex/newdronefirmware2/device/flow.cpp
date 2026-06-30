#include <string.h>
#include <stdlib.h>

#include "gpio.h"

#include "spi.h"
#include "tick.h"

#include "tof.h"

static void SPI1_TransferCons(const void* WriteData, unsigned long WriteSize, void* ReadData, unsigned long ReadSize)
{
  SPI1_CS_OF(true);

  SPI1_Transfer(WriteData, 0, WriteSize);
  SPI1_Transfer(0, ReadData, ReadSize);

  SPI1_CS_OF(false);
}
//------------------------------------------------------------------------------

static void SPI1_TransferParallel(const void* WriteData, void* ReadData, unsigned long Size)
{
  SPI1_CS_OF(true);

  SPI1_Transfer(WriteData, ReadData, Size);

  SPI1_CS_OF(false);
}
//------------------------------------------------------------------------------

static void WriteReg(char reg, char value)
{
  char send[2]={reg | 0x80, value};
  SPI1_TransferCons(send, 2, 0, 0);
}

bool FLOW_Init() // PMW3901
{
  SPI1_Init();
  
  WriteReg(0x3A, 0x5A);
  
  for(int a=0; a<100000; a++) { asm volatile("NOP"); }

  // Test the SPI communication, checking chipId and inverse chipId
  
  char reg[4]={0x00, 0, 0x5F, 0};
  SPI1_TransferParallel(reg, reg, 4);
  
  if (reg[1] != 0x49 || reg[3] != 0xB6) return false;
  
  for(int a=0; a<100000; a++) { asm volatile("NOP"); }
  
  WriteReg(0x7F, 0x00);
  WriteReg(0x61, 0xAD);
  WriteReg(0x7F, 0x03);
  WriteReg(0x40, 0x00);
  WriteReg(0x7F, 0x05);
  WriteReg(0x41, 0xB3);
  WriteReg(0x43, 0xF1);
  WriteReg(0x45, 0x14);
  WriteReg(0x5B, 0x32);
  WriteReg(0x5F, 0x34);
  WriteReg(0x7B, 0x08);
  WriteReg(0x7F, 0x06);
  WriteReg(0x44, 0x1B);
  WriteReg(0x40, 0xBF);
  WriteReg(0x4E, 0x3F);
  
  WriteReg(0x7F, 0x08);
  WriteReg(0x65, 0x20);
  WriteReg(0x6A, 0x18);
  WriteReg(0x7F, 0x09);
  WriteReg(0x4F, 0xAF);
  WriteReg(0x5F, 0x40);
  WriteReg(0x48, 0x80);
  WriteReg(0x49, 0x80);
  WriteReg(0x57, 0x77);
  WriteReg(0x60, 0x78);
  WriteReg(0x61, 0x78);
  WriteReg(0x62, 0x08);
  WriteReg(0x63, 0x50);
  WriteReg(0x7F, 0x0A);
  WriteReg(0x45, 0x60);
  WriteReg(0x7F, 0x00);
  WriteReg(0x4D, 0x11);
  WriteReg(0x55, 0x80);
  WriteReg(0x74, 0x1F);
  WriteReg(0x75, 0x1F);
  WriteReg(0x4A, 0x78);
  WriteReg(0x4B, 0x78);
  WriteReg(0x44, 0x08);
  WriteReg(0x45, 0x50);
  WriteReg(0x64, 0xFF);
  WriteReg(0x65, 0x1F);
  WriteReg(0x7F, 0x14);
  WriteReg(0x65, 0x60);
  WriteReg(0x66, 0x08);
  WriteReg(0x63, 0x78);
  WriteReg(0x7F, 0x15);
  WriteReg(0x48, 0x58);
  WriteReg(0x7F, 0x07);
  WriteReg(0x41, 0x0D);
  WriteReg(0x43, 0x14);
  WriteReg(0x4B, 0x0E);
  WriteReg(0x45, 0x0F);
  WriteReg(0x44, 0x42);
  WriteReg(0x4C, 0x80);
  WriteReg(0x7F, 0x10);
  WriteReg(0x5B, 0x02);
  WriteReg(0x7F, 0x07);
  WriteReg(0x40, 0x41);
  WriteReg(0x70, 0x00);
  
  for(int a=0; a<100000; a++) { asm volatile("NOP"); }
  
  WriteReg(0x32, 0x44);
  WriteReg(0x7F, 0x07);
  WriteReg(0x40, 0x40);
  WriteReg(0x7F, 0x06);
  WriteReg(0x62, 0xf0);
  WriteReg(0x63, 0x00);
  WriteReg(0x7F, 0x0D);
  WriteReg(0x48, 0xC0);
  WriteReg(0x6F, 0xd5);
  WriteReg(0x7F, 0x00);
  WriteReg(0x5B, 0xa0);
  WriteReg(0x4E, 0xA8);
  WriteReg(0x5A, 0x50);
  WriteReg(0x40, 0x80);
  
  return true;
}

bool FLOW_GetMotion(short& dX, short& dY, unsigned char& qual)
{
  char reg[8]={0x16,0,0,0,0,0,0,0};
  
  SPI1_CS_OF(true);
  
  SPI1_Transfer(reg, 0, 1);
  
  TICK_Delay(35);
  
  SPI1_Transfer(0, reg+1, 7);
  
  SPI1_CS_OF(false);
  
  short x = ((short)reg[4] << 8) | reg[3];
  short y = ((short)reg[6] << 8) | reg[5];
  
  dX = x;
  dY = -y;
  
  qual = reg[7];
  
  //bool obserf = reg[2] & 1<<5;
  //if(reg[2] & 1<<5) qual = 0;
  
  //if (qual == 0) asm("bkpt 0");
  
  return reg[1] != 0xFF;
}

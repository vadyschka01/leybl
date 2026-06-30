#include <device/bars/LPS22HH.h>
#include <math.h>

static void CallbackProc(unsigned char Address, const unsigned char* Data, unsigned char Size)
{
  BAR_Data result;

  const unsigned char* buff = Data;

  unsigned char st = buff[0];

  long temp_raw = 0;
  unsigned long press_raw = 0;
  press_raw = (((unsigned long)buff[3]) << 16) | (((unsigned long)buff[2]) << 8) | ((unsigned long)buff[1]);
  temp_raw = (((unsigned long)buff[5]) << 8) | ((unsigned long)buff[4]);

  if (st & 1)
  {
    result.Pressure = ((float)press_raw) * 0.0244140625f;
  }

  if (st & 2)
  {
    result.Temp = ((float)temp_raw) / 100.0f;
  }

  BAR_DoneProc(st == 3, result);
}

static unsigned char BAR_Buffer[32];

static  I2C_Request BAR_Device = { &CallbackProc, BAR_Buffer, sizeof(BAR_Buffer), 0 };

LPS22HH::LPS22HH()
{
  
}

LPS22HH::~LPS22HH()
{
  
}

void LPS22HH::Init()
{
  BAR_SetReg(Addr, 0x11, 0x04); // SWRESET

  for (int a = 0; a < 100000; a++) { asm volatile("NOP"); }

  BAR_SetReg(Addr, 0x10, 0x5A); // 100Hz | BDU | EN_LPFP | LPFP_CFG

  //BAR_SetReg(0x11, 0x12); // LOW_NOISE_EN | IF_ADD_INC

  for (int a = 0; a < 100000; a++) { asm volatile("NOP"); }
}

float LPS22HH::GetAltitude(float zero, float press, float temp)
{
  return (8.314f * (273.15f + temp) / (0.0289647f * 9.81f)) * logf(zero / press);
}

void LPS22HH::GetAsync(void (*DoneProc)(bool Ready, BAR_Data& Data))
{
  BAR_DoneProc = DoneProc;
  I2C1_Trans(&BAR_Device, Addr, &RegRead, 1, 6);
}

long LPS22HH::GetData(long* Temp)
{
  static float bar=0;
  static float temp=0;
  
  unsigned char st;
  I2C1_Write(Addr, 0x27);
  I2C1_Read(Addr, &st, 1);
  I2C1_Stop();
  
  if(st & 1)
  {
    unsigned char reg[3];
    I2C1_Write(Addr, 0x28);
    I2C1_Read(Addr, reg, sizeof(reg));
    I2C1_Stop();
    

    unsigned long b;

    b = reg[2];
    b = (b * 256U) + reg[1];
    b = (b * 256U) + reg[0];
    b *= 256U;
    
    bar=((float)b)/1048576.0f;
  }
  
  if(st & 2)
  {
    unsigned char reg[2];
    I2C1_Write(Addr, 0x2B);
    I2C1_Read(Addr, reg, sizeof(reg));
   I2C1_Stop();
    

    short t;

    t = (short)reg[1];
    t = (t * 256) + (short)reg[0];
    
    temp = (float) t / 100.0f;
  }
  
  *Temp=temp;
  
  return bar;
}
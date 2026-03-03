#include <math.h>

#include "i2c.h"

#include "bar.h"

#define NORMAL_MODE    0x03
#define FORCED_MODE    0x02

#define STANDBY_500US  0x00
#define STANDBY_10MS   0x06
#define STANDBY_20MS   0x07
#define STANDBY_6250US 0x01
#define STANDBY_125MS  0x02
#define STANDBY_250MS  0x03
#define STANDBY_500MS  0x04
#define STANDBY_1000MS 0x05

#define MODULE_DISABLE	0x00
#define OVERSAMPLING_1  0x01
#define OVERSAMPLING_2  0x02
#define OVERSAMPLING_4  0x03
#define OVERSAMPLING_8  0x04
#define OVERSAMPLING_16 0x05

#define FILTER_DISABLE  0x00
#define FILTER_COEF_2   0x01
#define FILTER_COEF_4   0x02
#define FILTER_COEF_8   0x03
#define FILTER_COEF_16  0x04

struct BAR_CalibrationData 
{												// Structure to store all calibration values
  unsigned short _T1;
  signed short   _T2;
  signed short   _T3;
  unsigned short _P1;
  signed short   _P2;
  signed short   _P3;
  signed short   _P4;
  signed short   _P5;
  signed short   _P6;
  signed short   _P7;
  signed short   _P8;
  signed short   _P9;
};

static BAR_CalibrationData CalibrationData;

static const unsigned char BAR_Addr = 0x76; // BMP280

static bool BAR2Init = false;
static const unsigned char BAR2_Addr = 0x5C; // LPS22HH


void (*BAR_DoneProc)(bool Ready, BAR_Data& Data);

float BAR_GetAltitude(float zero, float press, float temp)
{
  return (8.314f * (273.15f + temp) / (0.0289647f * 9.81f)) * logf(zero / press);
}
//------------------------------------------------------------------------------
void BAR1_CallbackProc(unsigned char Address, const unsigned char* Data, unsigned char Size)
{
  BAR_Data result;
  
  const unsigned char* buff=Data;
  
  long temp_raw=0;
  unsigned long press_raw=0;
  press_raw=(((unsigned long)buff[0]) << 16) | (((unsigned long)buff[1]) << 8) | ((unsigned long)buff[2]);
  temp_raw=(((unsigned long)buff[3]) << 16) | (((unsigned long)buff[4]) << 8) | ((unsigned long)buff[5]);
  
  if(press_raw==0x800000 || temp_raw==0x800000) { BAR_DoneProc(false, result); return; }
  
  temp_raw >>= 4;
  long value_t1 = ((((temp_raw >> 3) - ((long)CalibrationData._T1 << 1))) * ((long)CalibrationData._T2)) >> 11;
  long value_t2 = (((((temp_raw >> 4) - ((long)CalibrationData._T1)) * ((temp_raw >> 4) - ((long)CalibrationData._T1))) >> 12) * ((long)CalibrationData._T3)) >> 14;
  
  long temp=value_t1 + value_t2;
  
  long T = (temp * 5 + 128) >> 8;
  result.Temp = T / 100;

  press_raw >>= 4;
  long long value_1 = ((long long)temp) - 128000;
  long long value_2 = value_1 * value_1 * (long long)CalibrationData._P6;
  value_2 = value_2 + ((value_1 * (long long)CalibrationData._P5) << 17);
  value_2 = value_2 + (((long long)CalibrationData._P4) << 35);
  value_1 = ((value_1 * value_1 * (long long)CalibrationData._P3) >> 8) + ((value_1 * (long long)CalibrationData._P2) << 12);
  value_1 = (((((long long)1) << 47) + value_1)) * ((long long)CalibrationData._P1) >> 33;

  if(!value_1) { BAR_DoneProc(false, result); return; }

  long long p = 1048576 - press_raw;
  p = (((p << 31) - value_2) * 3125) / value_1;
  value_1 = (((long long)CalibrationData._P9) * (p >> 13) * (p >> 13)) >> 25;
  value_2 = (((long long)CalibrationData._P8) * p) >> 19;
  p = ((p + value_1 + value_2) >> 8) + (((long long)CalibrationData._P7) << 4);
  
  result.Pressure = p/256.0f;
  
  BAR_DoneProc(true, result);
}

void BAR2_CallbackProc(unsigned char Address, const unsigned char* Data, unsigned char Size)
{
  BAR_Data result;
  
  const unsigned char* buff=Data;
  
  unsigned char st=buff[0];
  
  long temp_raw=0;
  unsigned long press_raw=0;
  press_raw=(((unsigned long)buff[3]) << 16) | (((unsigned long)buff[2]) << 8) | ((unsigned long)buff[1]);
  temp_raw=(((unsigned long)buff[5]) << 8) | ((unsigned long)buff[4]);
  
  if(st & 1)
  {
    result.Pressure=((float)press_raw) * 0.0244140625f;
  }
  
  if(st & 2)
  {
    result.Temp = ((float) temp_raw) / 100.0f;
  }
  
  BAR_DoneProc(st==3, result);
}

void BAR_CallbackProc(unsigned char Address, const unsigned char* Data, unsigned char Size)
{
  if (BAR2Init) BAR2_CallbackProc(Address, Data, Size);
  else BAR1_CallbackProc(Address, Data, Size);
};
//------------------------------------------------------------------------------

static unsigned char BAR_Buffer[32];
static I2C_Request BAR_Device = {&BAR_CallbackProc, BAR_Buffer, sizeof(BAR_Buffer), 0};

static inline void BAR_SetReg(unsigned char Reg, unsigned char Value)
{
  unsigned char reg[2];
  reg[0]=Reg; reg[1]=Value;
  if (BAR2Init) I2C1_Write(BAR2_Addr, reg, 2);
  else I2C1_Write(BAR_Addr, reg, 2);
  I2C1_Stop();
}
//------------------------------------------------------------------------------

static void BAR_GetCalibration()
{
  I2C1_Write(BAR_Addr, 0x88);
  I2C1_Read(BAR_Addr, &CalibrationData, sizeof(CalibrationData));
  I2C1_Stop();
}
//------------------------------------------------------------------------------

bool BAR_Init() // BMP280
{
  I2C1_Init();

  if(!I2C1_CheckDevice(BAR_Addr))
  {
    if (!I2C1_CheckDevice(BAR2_Addr)) return false;
    BAR2Init = true;
  }
  if (BAR2Init)
  {
    BAR_SetReg(0x11, 0x04); // SWRESET
  
    for(int a=0; a<100000; a++) { asm volatile("NOP"); }
  
    BAR_SetReg(0x10, 0x5A); // 100Hz | BDU | EN_LPFP | LPFP_CFG
  
    //BAR_SetReg(0x11, 0x12); // LOW_NOISE_EN | IF_ADD_INC
  
    for(int a=0; a<100000; a++) { asm volatile("NOP"); }
  }
  else
  {
    BAR_SetReg(0xE0, 0xB6); // RESET
  
    for(int a=0; a<100000; a++) { asm volatile("NOP"); }
  
    BAR_GetCalibration();
  
    BAR_SetReg(0xF4, ((OVERSAMPLING_1 << 5) | (OVERSAMPLING_4 << 2) | NORMAL_MODE));
    BAR_SetReg(0xF5, ((STANDBY_500US << 5) | (FILTER_DISABLE << 2)));
  }
  return true;
}
//------------------------------------------------------------------------------

long BAR1_GetData(long* Temp)
{
  unsigned char buff[6];
  I2C1_Write(BAR_Addr, 0xF7);
  I2C1_Read(BAR_Addr, buff, sizeof(buff));
  I2C1_Stop();
  
  long temp_raw=0;
  unsigned long press_raw=0;
  press_raw=(((unsigned long)buff[0]) << 16) | (((unsigned long)buff[1]) << 8) | ((unsigned long)buff[2]);
  temp_raw=(((unsigned long)buff[3]) << 16) | (((unsigned long)buff[4]) << 8) | ((unsigned long)buff[5]);
  
  if(press_raw==0x800000 || temp_raw==0x800000) return 0;
  
  temp_raw >>= 4;
  long value_t1 = ((((temp_raw >> 3) - ((long)CalibrationData._T1 << 1))) * ((long)CalibrationData._T2)) >> 11;
  long value_t2 = (((((temp_raw >> 4) - ((long)CalibrationData._T1)) * ((temp_raw >> 4) - ((long)CalibrationData._T1))) >> 12) * ((long)CalibrationData._T3)) >> 14;
  
  long temp=value_t1 + value_t2;
  
  if(Temp) 
  {
    long T = (temp * 5 + 128) >> 8;
    *Temp = T / 100;
  }

  press_raw >>= 4;
  long long value_1 = ((long long)temp) - 128000;
  long long value_2 = value_1 * value_1 * (long long)CalibrationData._P6;
  value_2 = value_2 + ((value_1 * (long long)CalibrationData._P5) << 17);
  value_2 = value_2 + (((long long)CalibrationData._P4) << 35);
  value_1 = ((value_1 * value_1 * (long long)CalibrationData._P3) >> 8) + ((value_1 * (long long)CalibrationData._P2) << 12);
  value_1 = (((((long long)1) << 47) + value_1)) * ((long long)CalibrationData._P1) >> 33;

  if(!value_1) return 0;

  long long p = 1048576 - press_raw;
  p = (((p << 31) - value_2) * 3125) / value_1;
  value_1 = (((long long)CalibrationData._P9) * (p >> 13) * (p >> 13)) >> 25;
  value_2 = (((long long)CalibrationData._P8) * p) >> 19;
  p = ((p + value_1 + value_2) >> 8) + (((long long)CalibrationData._P7) << 4);
  
  return p / 256;
}

long BAR2_GetData(long* Temp)
{
  static float bar=0;
  static float temp=0;
  
  unsigned char st;
  I2C1_Write(BAR2_Addr, 0x27);
  I2C1_Read(BAR2_Addr, &st, 1);
  I2C1_Stop();
  
  if(st & 1)
  {
    unsigned char reg[3];
    I2C1_Write(BAR2_Addr, 0x28);
    I2C1_Read(BAR2_Addr, reg, sizeof(reg));
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
    I2C1_Write(BAR2_Addr, 0x2B);
    I2C1_Read(BAR2_Addr, reg, sizeof(reg));
   I2C1_Stop();
    

    short t;

    t = (short)reg[1];
    t = (t * 256) + (short)reg[0];
    
    temp = (float) t / 100.0f;
  }
  
  *Temp=temp;
  
  return bar;
  
}

long BAR_GetData(long* Temp)
{
  if (BAR2Init) return BAR2_GetData(Temp);
  else return BAR1_GetData(Temp);
}
//------------------------------------------------------------------------------

void BAR_GetAsunc(void (*DoneProc)(bool Ready, BAR_Data& Data))
{
  BAR_DoneProc=DoneProc;
  if (BAR2Init)
  {
    unsigned char reg=0x27;
    I2C1_Trans(&BAR_Device, BAR2_Addr, &reg, 1, 6);
  }
  else 
  {
    unsigned char reg=0xF7;
    I2C1_Trans(&BAR_Device, BAR_Addr, &reg, 1, 6);
  }
}
//------------------------------------------------------------------------------

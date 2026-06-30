#pragma once
#include <device/interfaces/IBar.h>

struct BMP280_CalibrationData
{
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

class BMP280 : IBar
{
public:
  BMP280();

  ~BMP280();

  unsigned char Addr = 0x76;

  unsigned char RegRead = 0xF7;

  void Init();

  float GetAltitude(float Zero, float Press, float Temp);

  void GetAsync(void (*DoneProc)(bool Ready, BAR_Data& Data));

  long GetData(long* Temp);
  
private:
  void GetCalibration();

  
};
#pragma once
#include <device/interfaces/IBar.h>

class LPS22HH : IBar
{
public:
  LPS22HH();

  ~LPS22HH();

  unsigned char Addr = 0x5C;

  unsigned char RegRead = 0x27;

  void Init();

  float GetAltitude(float Zero, float Press, float Temp);

  void GetAsync(void (*DoneProc)(bool Ready, BAR_Data& Data));

  long GetData(long* Temp);
};
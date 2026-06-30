#pragma once
#include "device/interfaces/IIMU.h"

class LSM6DS3TRC : IIMU
{
public:
  LSM6DS3TRC();
  
  ~LSM6DS3TRC();
  
  unsigned char IMU_Addr = 0x6A;

  unsigned char IMU_Read_Reg = 0x20;

  void Init();

  void IMU_SetBank(unsigned char Bank);

  void Get(IMU_Data& Data);

  void GetAsync(const IMU_Proc& DoneProc);
};
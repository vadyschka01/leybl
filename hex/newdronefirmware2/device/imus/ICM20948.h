#pragma once
#include "device/interfaces/IIMU.h"

class ICM20948 : IIMU
{
public:
  ICM20948();
  
  ~ICM20948();
  
  unsigned char IMU_Addr = 0x68;
  unsigned char IMU_Read_Reg = 0x2D;

  unsigned char MAG_Addr = 0x0C; // может не быть

  void Init();
  void IMU_SetBank(unsigned char Bank);
  unsigned char MAG_GetReg(unsigned char Slave, unsigned char Reg, unsigned char Len = 0);
  void MAG_SetReg(unsigned char Slave, unsigned char Reg, unsigned char Value);
  void Get(IMU_Data& Data);
  void GetAsync(const IMU_Proc& DoneProc);
};
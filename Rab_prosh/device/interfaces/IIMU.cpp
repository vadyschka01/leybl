#pragma once
#include "device/interfaces/IIMU.h"
#include "device/imus/ICM20948.h"
#include "device/imus/LSM6DS3TRC.h"
#include "device/imus/ICM40609.h"

IMU_Info IMUInfo;

typedef void (*IMU_Proc)(IMU_Data&);

IMU_Calib_Data CalibDataIMU;

void IMU_SetReg(unsigned char Addr, unsigned char Reg, unsigned char Value)
{
  unsigned char reg[2];
  reg[0] = Reg; reg[1] = Value;
  I2C1_Write(Addr, reg, 2);
  I2C1_Stop();
}

unsigned char IMU_GetReg(unsigned char Addr, unsigned char Reg)
{
  I2C1_Write(Addr, Reg);
  I2C1_Read(Addr, &Reg, 1);
  I2C1_Stop();
  return Reg;
}

IIMU* TryFindIMU(bool& find)
{
  I2C1_Init();
  IIMU* Imu = nullptr;
  for (int i = 0; i < sizeof(IMUInfo.Addrs); i++)
  {
    find = I2C1_CheckDeviceWhoAmI(IMUInfo.Addrs[i], IMUInfo.WhoIAmReg[i], IMUInfo.ExpectedIDs[i]);
    if (find && i == 0)
    {
      static ICM20948 imuICM20948;
      Imu = (IIMU*)&imuICM20948;
      break;
    }
    else if (find && i == 1)
    {
      static LSM6DS3TRC imuLSM6DS3TRC;
      Imu = (IIMU*)&imuLSM6DS3TRC;
      break;
    }
    else if (find && i == 2)
    {
      static ICM40609 imuICM40609;
      Imu = (IIMU*)&imuICM40609;
      break;
    }
    for (int a = 0; a < 10000000; a++) { asm volatile("NOP"); }
  }
  if (find) Imu->Init();
  return Imu;
}
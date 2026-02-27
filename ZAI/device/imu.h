#pragma once

struct IMU_Data
{
  long Temp;
  struct { float X, Y, Z; } Acc;
  struct { float X, Y, Z; } RawAcc;
  struct { float X, Y, Z; } Gyr;
  struct { float X, Y, Z; } RawGyr;
  struct { float X, Y, Z; } Mag;
  struct { float X, Y, Z; } RawMag;
};
#pragma pack(push,1)
struct XYZ_Calib
{
  short X[2] = {0,0};
  short Y[2] = {0,0};
  short Z[2] = {0,0};
};

struct XYZ_IMU_DATA
{
  char Writed = sizeof(XYZ_IMU_DATA);
  XYZ_Calib Acc;
  XYZ_Calib Mag;
};

struct IMU_Calib_Data
{
  bool AllowedCalib = false;
  bool CalibAccNeed = true;
  bool CalibMagNeed = true;
  XYZ_IMU_DATA Data;
};
#pragma pack(pop)

typedef void (*IMU_Proc)(IMU_Data&);
extern IMU_Calib_Data CalibDataIMU;
bool IMU_Init();
void IMU_Get(IMU_Data& Data);
void IMU_GetAsunc(const IMU_Proc& DoneProc);

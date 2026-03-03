#pragma once

struct MAG_Data
{
  float X, Y, Z;
  float RawX, RawY, RawZ;
};

struct MAG_Calib_Data
{
  bool CalibNeed = true;
  short X[2] = {0,0};
  short Y[2] = {0,0};
  short Z[2] = {0,0};
};

bool MAG_Init();
bool MAG_Get(MAG_Data& Data);
void MAG_GetAsunc(void (*DoneProc)(MAG_Data& Data));

extern MAG_Calib_Data ExternMagCalibData;

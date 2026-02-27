#pragma once


#define JOY_MAX 2000
#define JOY_MID 1000

struct SBUS_Data // FlySky
{
  short X, Y, Z, W = 0;
  short SWA, SWB, SWC, SWD = 0;
  short VRA, VRB = 0;
  short LastSWA, LastSWB, LastSWC, LastSWD = 0;
  short OTHER[8] = {0,0,0,0,0,0,0,0};
  
  bool Active = 0;
  bool FrameLost = 0;
  bool FailSafe = 0;
};

void SBUS_Init();
bool SBUS_Update(SBUS_Data& Data);

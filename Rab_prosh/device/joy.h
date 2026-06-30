#pragma once

#define JOYPAD_MAX 2000
#define JOYPAD_MID 1000

enum class JOPAD_TYPE {NO = 0, SBUS = 1, CRSF = 2};

struct JOYPAD_Data // Crossfire
{
  short X, Y, Z, W = 0;
  short SWA, SWB, SWC, SWD = 0;
  short VRA, VRB = 0;
  short LastSWA, LastSWB, LastSWC, LastSWD = 0;
  short OTHER[6] = {0,0,0,0,0,0};
  
  bool Active = 0;
  bool FrameLost = 0;
  bool FailSafe = 0;
};
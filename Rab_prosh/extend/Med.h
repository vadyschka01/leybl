#pragma once

struct MED_Data16
{
  short Size, Index;
  short *Buf, *Temp;
};

struct MED_Data32
{
  long Size, Index;
  long *Buf, *Temp;
};

struct MED_DataF32
{
  long Size, Index;
  float *Buf, *Temp;
};

short MED_Update(short Value, MED_Data16& Data);
long MED_Update(long Value, MED_Data32& Data);
float MED_Update(float Value, MED_DataF32& Data);

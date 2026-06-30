#include <string.h>

#include "med.h"

short MED_Update(short Value, MED_Data16& Data)
{
  Data.Buf[Data.Index++] = Value;
  if (Data.Index >= Data.Size) Data.Index = 0;

  memcpy(Data.Temp, Data.Buf, Data.Size * sizeof(short));

  short* t = Data.Temp;
  short s = Data.Size;

  for (long a = 0; a < s; a++)
  for (long b = a; b < s; b++)
  {
    if (t[a] <= t[b]) continue;
    short v = t[a];
    t[a] = t[b];
    t[b] = v;
  }

  return t[s / 2];
}
//------------------------------------------------------------------------------

long MED_Update(long Value, MED_Data32& Data)
{
  Data.Buf[Data.Index++] = Value;
  if (Data.Index >= Data.Size) Data.Index = 0;

  memcpy(Data.Temp, Data.Buf, Data.Size * sizeof(long));

  long* t = Data.Temp;
  long s = Data.Size;

  for (long a = 0; a < s; a++)
  for (long b = a; b < s; b++)
  {
    if (t[a] <= t[b]) continue;
    long v = t[a];
    t[a] = t[b];
    t[b] = v;
  }

  return t[s / 2];
}
//------------------------------------------------------------------------------

float MED_Update(float Value, MED_DataF32& Data)
{
  Data.Buf[Data.Index++] = Value;
  if (Data.Index >= Data.Size) Data.Index = 0;

  memcpy(Data.Temp, Data.Buf, Data.Size * sizeof(float));

  float* t = Data.Temp;
  long s = Data.Size;

  for (long a = 0; a < s; a++)
  for (long b = a; b < s; b++)
  {
    if (t[a] <= t[b]) continue;
    float v = t[a];
    t[a] = t[b];
    t[b] = v;
  }

  return t[s / 2];
}
//------------------------------------------------------------------------------

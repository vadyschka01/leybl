#pragma once

struct BAR_Data
{
  long Temp;
  float Pressure;
};

bool BAR_Init();

long BAR_GetData(long* Temp);
void BAR_GetAsunc(void (*DoneProc)(bool Ready, BAR_Data& Data));
float BAR_GetAltitude(float p0, float p1);

#pragma once

struct BAR_Data
{
  long Temp;
  float Pressure;
};

void BAR_Init();

float BAR_GetData(float* Temp);
void BAR_GetAsunc(void (*DoneProc)(bool Ready, BAR_Data& Data));
float BAR_GetAltitude(float p0, float p1);

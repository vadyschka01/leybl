#pragma once

struct CamFlowData
{
  float X;
  float Y;
  float YAW;
};

bool CamFlow_Update();
void CamFlow_Init();
bool CamFlow_GetData(CamFlowData& data);
void CamFlow_Send(float range);
#pragma once

float Deg2Rad(float Deg);
float Rad2Deg(float Rad);
float FastAtan2f(float y, float x);
float GetLocalDistance(float p1Lat, float p1Lon, float p2Lat, float p2Lon, float& dx, float& dy);
void GlobalToLocal(const float& Plat, const float& Plon, const float& Olat, const float& Olon, float& dx, float& dy);
void LocalToGlobal(float x, float y, const float& Olat, const float& Olon, float& lat, float& lon);
float Normalize(float Value, float Scale, float Min, float Max);
short Rev16(short v);
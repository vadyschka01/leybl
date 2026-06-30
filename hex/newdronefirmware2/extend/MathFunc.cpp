#include <math.h>
#include "MathFunc.h"

static const float PI = 3.14159265359;
static const float EARTH_RADIUS = 6371000.0f; // Radius of the earth in m

float FastAtan2f(float y, float x)
{
  float r, angle, abs_y = fabsf(y) + 1e-10f;
  if (x < 0.0f) {
    r = (x + abs_y) / (abs_y - x);
    angle = 3.0 * PI / 4.0;
  }
  else {
    r = (x - abs_y) / (x + abs_y);
    angle = PI / 4.0;
  }
  angle += (0.1963f * r * r - 0.9817f) * r;
  return (y < 0.0f) ? -angle : angle;
}
// ѕолучение дистанции (в локальной системе) между двум€ глобальными координатами
float GetLocalDistance(float p1Lat, float p1Lon, float p2Lat, float p2Lon, float& dx, float& dy) // light formula
{
  dy = EARTH_RADIUS / 360.0f * ((p1Lat - p2Lat) * PI * 2.0f); // lat
  dx = cosf(((p1Lat + p2Lat) / 2.0f) * PI / 180.0f) * EARTH_RADIUS / 360.0f * ((p1Lon - p2Lon) * PI * 2.0f); // long
  return sqrtf(dx * dx + dy * dy);
}
//  онвертаци€ градусов в радианы
float Deg2Rad(float Deg)
{
  return Deg * PI / 180.0f;
}
//  онвертаци€ радиан в градусы
float Rad2Deg(float Rad)
{
  return Rad * 180.0f / PI;
}
// ѕеревод глобальных координат в локальные (относительно нулевой точки)
void GlobalToLocal(const float& Plat, const float& Plon, const float &Olat, const float &Olon, float& dx, float& dy)
{
  dy = EARTH_RADIUS * Deg2Rad(Plon - Olon) * cosf(Deg2Rad(Olat));
  dx = EARTH_RADIUS * Deg2Rad(Plat - Olat);
  return;
}
// ѕеревод локальных координат в глобальные (относительно нулевой точки)
void LocalToGlobal(float x, float y, const float &Olat, const float &Olon, float &lat, float &lon)
{
  // ¬ычисление новой широты
  lat = Rad2Deg(Deg2Rad(Olat) + (x / EARTH_RADIUS));
  // ¬ычисление новой долготы (с использованием средней широты дл€ точности)
  lon = Olon + Rad2Deg(y / (EARTH_RADIUS * cosf((Deg2Rad(Olat) + (Deg2Rad(Olat) + (x / EARTH_RADIUS))) / 2.0)));
  return;
}

short Rev16(short v)
{
  asm("REV16 %1, %0" : "=r" (v) : "r" (v)); // v = v<<8 | v>>8;
  return v;
}

float Normalize(float Value, float Scale, float Min, float Max)
{
  const float len = (Max - Min) / 2.0f;
  const float shift = (Max + Min) / 2.0f;
  return (Value - shift) * Scale / len;
}
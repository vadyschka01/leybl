#include "RegPDI.h"

static inline float sat(float val, float min, float max)
{
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

static inline float abs(float val)
{
  if (val < 0) return -val;
  return val;
}

float RegPDI::Update(float Value, float Current, float Derivative)
{
  float e = Value - Current;

  float p = e * Pro;

  float d = Derivative * Der;

  float ip = e * Int.Pro * Period;
  ip = sat(ip, -Int.Lim, Int.Lim);

  float id = Derivative * Int.Der;

  I += ip + id;

  float l = 0;
  if (Int.Lim > abs(p)) l = Int.Lim - abs(p);

  I = sat(I, -l, l);

  if (LockI) I = 0;

  return sat(p + d + I, -Lim, Lim);
}

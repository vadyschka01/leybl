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

  if(!LockI)
  {
    float c = abs(p) + abs(d);
    if(c > Lim) c = Lim;
    
    c = 1.0f - Lim/c;
    
    I += e * Int.Pro * c;
    I = sat(I, -Int.Lim, Int.Lim);
  }
  
  return sat(p + d + I, -Lim, Lim);
}

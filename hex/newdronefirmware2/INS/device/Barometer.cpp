#include <math.h>

#include "Barometer.h"

static constexpr float G = 9.80665; // m/c2

static float sat(float value, float max, float min)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

Barometer::Barometer(const Record<Vector3>& RecordPos, const Record<Vector3>& RecordSpd, const Record<Quaternion>& RecordQua) :
  RecPos(RecordPos), RecSpd(RecordSpd)
{
  Active = false;
}

void Barometer::UpdateAverage()
{
  if (!Active) return;

  AveSpd.Set(RecSpd.Past(Past).Z);
}

void Barometer::SetPressure(const float* Pressure, const float* Temperature)
{
  if (Temperature) Temp = *Temperature;

  if (!Pressure || !Active)
  {
    if (Active) AveSpd.Get();
    Active = Pressure;
    return;
  }

  Bar = *Pressure;

  if (Zero == 0.0f) Zero = Bar;

  float position = (8.314f * (273.15f + Temp) / (0.0289647f * 9.81f)) * logf(Zero / Bar);
  float speed = (position - LastPos) / Period;
  
  LastPos = position;

  float pos = position - RecPos.Past(Past).Z;
  float spd = speed - AveSpd.Get();
  
  
  
  //---
  float d_spd = sat(fabsf((spd - Shift.Prv)), RegShift.Accuracy, 0.0f);

  float r_spd = RegShift.Restore * Period;
  if (r_spd > 1.0f) r_spd = 1.0f;

  if (Shift.Dyn < d_spd) Shift.Dyn = d_spd;
  else Shift.Dyn = Shift.Dyn*(1.0f-r_spd) + d_spd*r_spd;

  float alpha = (1.0f - Shift.Dyn / RegShift.Accuracy);
  
  alpha=powf(alpha, RegShift.Power);
  
  RegShift.Test=alpha;
  
  float a_pos = alpha * RegPos.Dynamic;
  float a_spd = alpha * RegSpd.Dynamic;
  
  float s_pos = sat(fabsf(pos), RegPos.Distance, 0.0f) / RegPos.Distance;
  float s_spd = sat(fabsf(spd), RegSpd.Distance, 0.0f) / RegSpd.Distance;
  
  a_pos += (powf(s_pos, RegPos.Power) * RegPos.Alpha) * alpha;
  a_spd += (powf(s_spd, RegSpd.Power) * RegSpd.Alpha) * alpha;
  
  
  
  /*float min = 0.0f;
  if(spd<Adaptive.AccuracyMin) min = 1.0f - fabsf(spd)/Adaptive.AccuracyMin;
  
  float a_pos=sat(min, Adaptive.AlphaMax, Adaptive.AlphaMin);
  float a_spd=sat(min, Adaptive.AlphaMax, Adaptive.AlphaMin);*/
  //---
  
  Shift.Pos = pos * a_pos;
  Shift.Spd = spd * a_spd;

  Shift.Prv = spd;

  Shift.Ready = true;
}

void Barometer::UpdateZeroHeight(float ZeroHeight, float Alpha)
{
  float height = ZeroHeight;
  
  //LastPos = height;
  
  float alpha = fabsf(LastPos - ZeroHeight);
  if (alpha>Distance) alpha = 1.0f;
  else alpha = powf(alpha / Distance, Power);
  
  
  float zero = Bar * expf((9.81f * 0.0289647f * height) / (8.314f * (273.15f + Temp)));

  if (Alpha<1.0f && Alpha>0.0f) alpha*=Alpha;
  
  Zero = Zero * (1.0f - alpha) + zero * alpha;
}

void Barometer::GetTempBarZero(float* Temperature, float* Pressure, float* ZeroPressure)
{
  if (Temperature) *Temperature = Temp;
  if (Pressure) *Pressure = Bar;
  if (ZeroPressure) *ZeroPressure = Zero;
}

bool Barometer::GetShift(float& PositionZ, float& SpeedZ)
{ 
  bool ready = Shift.Ready;

  PositionZ = Shift.Pos;
  SpeedZ = Shift.Spd;
  
  if (ready) Shift.Prv -= Shift.Spd;

  Shift.Ready = false;

  return ready;
}

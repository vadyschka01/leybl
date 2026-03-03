#include <math.h>

#include "Range.h"

static constexpr float PI = 3.14159265359f;
static constexpr float TO_DEG = 180.0f / PI;
static constexpr float TO_RAD = PI / 180.0f;

static float sat(float value, float max, float min)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

Range::Range(const Record<Vector3>& RecordSpd, const Record<Quaternion>& RecordQua) :
  RecQua(RecordQua),
  RecSpd(RecordSpd)
{

}

void Range::UpdateAverage()
{
  if (!Active) return;

  AveSpd.Set(RecSpd.Past(Past));
}

float Range::SetRange(float* Range)
{
  Quaternion qua = RecQua.Past(Past);
  Quaternion tilt = qua * Quaternion(0, 0, 1, 0) * qua.Conjugate();
  tilt = tilt.Norm();
  
  if (tilt.Z<MaxAngle) Range = nullptr;
  
  if (Range)
  {
    if (*Range > MaxHeight) Range = nullptr;
    else if (*Range < MinHeight) Range = nullptr;
  }
  
  if (!Range || !Active)
  {
    if (Active) 
    {
      AveSpd.Get();
    }
    
    float len = -1;
    
    if (Range) Len = len = *Range * tilt.Z;
    
    Active = Range;
    return len;
  }
  
  if (!Range) return -1;
  
  float len = *Range * tilt.Z;
  
  float last=Len;
  Len=len;
  
  float speed = (len - last) * Freq;
  
  Vector3 ave = AveSpd.Get();
  
  float spd_xy = Vector2(ave).Length();
  if(spd_xy<MaxSpeedXY) spd_xy = 1.0f - spd_xy/MaxSpeedXY;
  else spd_xy = 0.0f;
  
  float spd = speed - ave.Z;
  
  float d_spd = fabsf(spd - Shift.Prv);
  if (d_spd > RegShift.Accuracy) d_spd = RegShift.Accuracy; else if (d_spd < 0.0f) d_spd = 0.0f;

  float r_spd = RegShift.Restore * Period;
  if (r_spd > 1.0f) r_spd = 1.0f;

  if (Shift.Dyn < d_spd) Shift.Dyn = d_spd;
  else Shift.Dyn = Shift.Dyn*(1.0f-r_spd) + d_spd*r_spd;

  float alpha = (1.0f - Shift.Dyn / RegShift.Accuracy);
  
  alpha = powf(alpha, RegShift.Power);
  
  RegShift.Test=alpha;
  
  float a_spd = alpha * RegSpd.Dynamic;
  
  float al_spd=fabsf(spd);
  if (al_spd > RegSpd.Distance) al_spd = RegSpd.Distance; else if (al_spd < 0.0f) al_spd = 0.0f;
  
  float s_spd = al_spd / RegSpd.Distance;
  
  a_spd += powf(s_spd, RegSpd.Power) * RegSpd.Alpha * alpha;
  
  if (a_spd > 1.0f) a_spd = 1.0f; else if (a_spd < 0.0f) a_spd = 0.0f;
  
  
  Shift.Spd = spd * a_spd * spd_xy;

  Shift.Prv = spd;

  Shift.Ready = true;
  
  return Len;
}

bool Range::GetShift(float& SpeedZ)
{
  bool ready = Shift.Ready;

  SpeedZ = Shift.Spd;

  if (ready) Shift.Prv -= Shift.Spd;

  Shift.Ready = false;

  return ready;
}

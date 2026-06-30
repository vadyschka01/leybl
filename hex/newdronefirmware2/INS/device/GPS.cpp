#include <math.h>

#include "GPS.h"

static constexpr float PI = 3.14159265359f;
static constexpr float TO_DEG = 180.0f / PI;
static constexpr float TO_RAD = PI / 180.0f;

GPS::GPS(const Record<Vector3>& RecordPos, const Record<Vector3>& RecordSpd) :
  RecPos(RecordPos), RecSpd(RecordSpd)
{
  Active = false;
}

void GPS::UpdateAverage(float Time)
{
  if (!Active) return;

  if (Time - Timer > Wait)
  {
    Active = false;
    return;
  }

  AveSpd.Set(RecSpd.Past(Past));
}

void GPS::SetPosition(const Vector3* Position, float Time)
{
  if (!Position || !Active)
  {
    if (Active) AveSpd.Get();
    Active = Position;
    if (Active)
    {
      Timer = Time;
      LastPos = *Position;
    }
    return;
  }

  const Vector3& cur = *Position;

  float dt = Time - Timer;

  LastSpd = (cur - LastPos) / dt;
  LastPos = cur;
  Timer = Time;

  Vector3 pos = LastPos - RecPos.Past(Past);
  Vector3 spd = LastSpd - AveSpd.Get();
  
  Vector3 d_spd = (spd - Shift.Prv).Abs().Limit(RegShift.Accuracy, 0.0f);

  float r_spd = RegShift.Restore * Period;
  if (r_spd > 1.0f) r_spd = 1.0f;

  if (Shift.Dyn.X < d_spd.X) Shift.Dyn.X = d_spd.X;
  else Shift.Dyn.X = Shift.Dyn.X*(1.0f-r_spd) + d_spd.X*r_spd;
  
  if (Shift.Dyn.Y < d_spd.Y) Shift.Dyn.Y = d_spd.Y;
  else Shift.Dyn.Y = Shift.Dyn.Y*(1.0f-r_spd) + d_spd.Y*r_spd;
  
  if (Shift.Dyn.Z < d_spd.Z) Shift.Dyn.Z = d_spd.Z;
  else Shift.Dyn.Z = Shift.Dyn.Z*(1.0f-r_spd) + d_spd.Z*r_spd;

  Vector3 alpha = (1.0f - Shift.Dyn / RegShift.Accuracy);
  
  alpha = alpha.Power(RegShift.Power);
  
  RegShift.Test=alpha.X;
  
  Vector3 a_pos = alpha * RegPos.Dynamic;
  Vector3 a_spd = alpha * RegSpd.Dynamic;
  
  //Vector3 s_pos = pos.Abs().Limit(RegPos.Distance, 0.0f) / RegPos.Distance;
  //Vector3 s_spd = spd.Abs().Limit(RegSpd.Distance, 0.0f) / RegSpd.Distance;
  
  //a_pos += a_pos.Power(RegPos.Power) * Vector3(RegPos.Alpha, RegPos.Alpha, AlphaHeight) * alpha; // Сомнительно
  //a_spd += s_spd.Power(RegSpd.Power) * Vector3(RegSpd.Alpha, RegSpd.Alpha, AlphaHeight) * alpha; // Сомнительно
  
  a_pos=a_pos.Limit(1.0f, 0.0f);
  a_spd=a_spd.Limit(1.0f, 0.0f);
  
  //a_pos = 0.2f;//RegPos.Alpha;
  //a_spd = 0.1f;//RegSpd.Alpha;
  
  Shift.Pos = pos * a_pos;
  Shift.Spd = spd * a_spd;
  
  if(Shift.Dyn.IsNAN()) Shift.Dyn=0;
  if(Shift.Spd.IsNAN()) Shift.Spd=0;
  if(Shift.Pos.IsNAN()) Shift.Pos=0;
  if(Shift.Prv.IsNAN()) Shift.Prv=0;
  
  Shift.Prv = spd;
  
  Shift.Ready = true;
}

bool GPS::GetTime(float& Time)
{
  Time = Timer;

  return Active;
}

bool GPS::GetPositionSpeed(Vector3* Position, Vector3* Speed)
{
  if (Speed) *Speed = LastSpd;
  if (Position) *Position = LastPos;

  return Active;
}

bool GPS::GetShift(Vector3& Position, Vector3& Speed)
{
  bool ready = Shift.Ready;
  
  Position=Shift.Pos;
  Speed=Shift.Spd;
  
  if(ready) Shift.Prv -= Shift.Spd;
  
  Shift.Ready=false;
  
  return ready;
}

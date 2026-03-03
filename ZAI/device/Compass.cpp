#include <math.h>

#include "Compass.h"

static constexpr float PI = 3.14159265359f;
static constexpr float TO_DEG = 180.0f / PI;
static constexpr float TO_RAD = PI / 180.0f;

Compass::Compass(const Record<Vector2>& RecordOF, const Record<Vector3>& RecordGyr, const Record<Quaternion>& RecordQua) :
  RecOF(RecordOF), RecGyr(RecordGyr), RecQua(RecordQua)
{
  Active = false;
}

void Compass::UpdateAverageOF()
{
  Vector2 v = RecOF.Past(Past);

  if (v.IsNAN()) Active = false;
  else AveSpd.Set(v);
}

void Compass::SetMagnet(const Vector3& Mag)
{
  
}

void Compass::SetCourseGPS(const Vector2& SpeedGPS, float Time)
{
  float dt = Time - Timer;
  Timer = Time;

  if (dt > MaxTimerGPS)
  {
    Active = false;
    return;
  }

  Vector2 of = AveSpd.Get();

  if (!Active)
  {
    Active = true;
    return;
  }

  Vector2 gps = SpeedGPS;

  float len_of = of.Length();
  float len_gps = gps.Length();

  float alpha = fminf(len_of, len_gps);

  if (alpha > CourseSpeed) alpha = 1.0f;
  else alpha /= CourseSpeed;
  alpha *= alpha;

  gps = gps.Norm();
  of = of.Norm();

  float yaw_of = atan2f(of.Y, of.X);
  float yaw_gps = atan2f(gps.Y, gps.X);

  Shift.Yaw = (yaw_gps - yaw_of) * TO_DEG * alpha;

  Shift.Ready = true;
}

bool Compass::GetShift(float& Yaw)
{
  bool ready = Shift.Ready;
  Shift.Ready = false;

  Yaw = Shift.Yaw;

  return ready;
}

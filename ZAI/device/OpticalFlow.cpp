#include <math.h>

#include "OpticalFlow.h"

static constexpr float PI = 3.14159265359f;
static constexpr float TO_DEG = 180.0f / PI;
static constexpr float TO_RAD = PI / 180.0f;

OpticalFlow::OpticalFlow(const Record<Vector3>& RecordGyr, const Record<Vector3>& RecordSpd, const Record<Vector3>& RecordPos, const Record<Quaternion>& RecordQua) :
  RecGyr(RecordGyr), RecSpd(RecordSpd), RecPos(RecordPos), RecQua(RecordQua), RecordOF(RecOF.Rec)
{
  Active = false;
  RecordOF.Init(RecOF.Buffer, Freq, RecOF.Past);
}

void OpticalFlow::UpdateAverage(float InertialPeriod)
{
  if (!Active) return;

  AveGyrMove += (RecGyr.Past(Past)) * InertialPeriod;
  AveSpd.Set(RecSpd.Past(Past));
}

void OpticalFlow::SetMove(const Vector2* Move, float Range, float Quality)
{
  float length = Range;
  if (length < 0.01f) length = 0.01f;
  
  //length=0.5f;
  
  if (!Move || !Active)
  {
    if (Active)
    {
      AveGyrMove.Zero();
      AveSpd.Get();
    }
    Active = Move;

    RecordOF.Add(Vector2(NAN, NAN));
    return;
  }
  
  Quaternion qua = RecQua.Past(Past);
  Quaternion tilt = qua * Quaternion(0, 0, 1, 0) * qua.Conjugate();
  tilt = tilt.Norm();
  
  length /= tilt.Z;

  Vector2 gyr = { AveGyrMove.Y, -AveGyrMove.X };

  Vector2 move = *Move;

  move *= Lens; // компенсация линзы
  
  TEST_SHIFT.X=move.X;
  TEST_SHIFT.Y=-gyr.X;
  
  TEST_SHIFT *= 100;
  
  move += gyr; // компенсация наклона
  
  move *= length;
  move *= Freq;
  move = qua.RotateAroundZ(move);

  RecordOF.Add(move);

  AveGyrMove.Zero();
  
  const float limit = 0.5f; // абсолютный наклон 60 градусов
  const float t=acosf(tilt.Z)*TO_DEG;
  
  float angle = 1.0f; // Влияние наклонов
  if(tilt.Z>limit && t<MaxAngle)
  {
    if(t>MinAngle) 
      angle=1.0f-(t-MinAngle)/(MaxAngle-MinAngle);
  }
  else angle=0.0f;
    
  Vector2 rotate = gyr.Abs();
  
  Vector2 speed = 1.0f; // Влияние угловой скорости
  
  if(rotate.X>MinSpeed)
  {
    if(rotate.X<MaxSpeed) speed.X=1.0f-(rotate.X-MinSpeed)/(MaxSpeed-MinSpeed);
    else speed.X=0.0f;
  }
  
  if(rotate.Y>MinSpeed)
  {
    if(rotate.Y<MaxSpeed) speed.Y=1.0f-(rotate.Y-MinSpeed)/(MaxSpeed-MinSpeed);
    else speed.Y=0.0f;
  }
  
  float quality = 1.0f; // Влияние качества оптического потока
  
  if(Quality<MaxQuality)
  {
    if(Quality>MinQuality) quality=(Quality-MinQuality)/(MaxQuality-MinQuality);
    else quality=0.0f;
  }
  
  float height = 1.0f; // Влияние высоты оптического потока
  
  if(length>MinHeight)
  {
    if(length<MaxHeight) height=1.0f-(length-MinHeight)/(MaxHeight-MinHeight);
    else height=0.0f;
  }
  
  Vector2 trust = speed * angle * quality * height;
  
  Vector2 spd = (move - AveSpd.Get()) * trust;
  
  Vector2 d_spd = (spd - Shift.Prv).Abs().Limit(RegShift.Accuracy, 0.0f);

  float r_spd = RegShift.Restore * Period;
  if (r_spd > 1.0f) r_spd = 1.0f;

  if (Shift.Dyn.X < d_spd.X) Shift.Dyn.X = d_spd.X;
  else Shift.Dyn.X = Shift.Dyn.X*(1.0f-r_spd) + d_spd.X*r_spd;
  
  if (Shift.Dyn.Y < d_spd.Y) Shift.Dyn.Y = d_spd.Y;
  else Shift.Dyn.Y = Shift.Dyn.Y*(1.0f-r_spd) + d_spd.Y*r_spd;

  Vector2 alpha = (1.0f - Shift.Dyn / RegShift.Accuracy);
  
  alpha = alpha.Power(RegShift.Power);
  
  RegShift.Test=alpha.X;
  
  Vector2 a_spd = alpha * RegSpd.Dynamic;
  
  Vector2 s_spd = spd.Abs().Limit(RegSpd.Distance, 0.0f) / RegSpd.Distance;
  
  a_spd += s_spd.Power(RegSpd.Power) * RegSpd.Alpha * alpha;
  
  a_spd=a_spd.Limit(1.0f, 0.0f);
  
  //a_spd=1.0f;

  Shift.Spd  = spd * a_spd * trust;
  Shift.Pos = spd * trust * Period;

  Shift.Prv = spd;

  Shift.Ready = true;
}

bool OpticalFlow::GetShift(Vector2& Position, Vector2& Speed)
{
  bool ready = Shift.Ready;

  Position = Shift.Pos;
  Speed = Shift.Spd;

  if (ready) Shift.Prv -= Shift.Spd;

  Shift.Ready = false;

  return ready;
}

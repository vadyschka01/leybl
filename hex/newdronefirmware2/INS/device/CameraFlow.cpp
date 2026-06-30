#include <math.h>

#include "CameraFlow.h"

CameraFlow::CameraFlow(const Record<Vector3>& RecordSpd, const Record<Vector3>& RecordPos, const Record<Quaternion>& RecordQua) :
  RecSpd(RecordSpd), RecPos(RecordPos), RecQua(RecordQua)
{
  Active = false;
}

void CameraFlow::UpdateAverage()
{
  if (!Active) return;

  AveSpd.Set(RecSpd.Past(0));
}

void CameraFlow::SetMove(const Vector2* Move, float Range, unsigned long Time)
{
  float period=((float)(Time-LastTime))/1000.0f;
  LastTime=Time;
  
  float length = Range;
  if (length < 0.01f) length = 0.01f;
  
  if (!Move || !Active)
  {
    if (Active)
    {
      AveSpd.Get();
    }
    Active = Move;

    //Vector2 sub = BufferPos * BufferAlpha;
    //BufferPos-=sub;
    //Shift.Pos = sub;
    
    return;
  }
  
  Quaternion qua = RecQua.Past(0);
  Quaternion tilt = qua * Quaternion(0, 0, 1, 0) * qua.Conjugate();
  tilt = tilt.Norm();
  
  length /= tilt.Z;

  Vector2 move = *Move;

  move *=  Lens * length; // компенсация линзы
  
  move /= period;
  
  move = qua.RotateAroundZ(move);
  
  Vector2 spd = (move - AveSpd.Get());
  
  if(spd.IsNAN()) return;
  
  //ShiftSpd=spd;
  
  Vector2 d_spd = (spd - Shift.Prv).Abs().Limit(RegShift.Accuracy, 0.0f);

  float r_spd = RegShift.Restore * period;
  if (r_spd > 1.0f) r_spd = 1.0f;

  if (Shift.Dyn.X < d_spd.X) Shift.Dyn.X = d_spd.X;
  else Shift.Dyn.X = Shift.Dyn.X*(1.0f-r_spd) + d_spd.X*r_spd;
  
  if (Shift.Dyn.Y < d_spd.Y) Shift.Dyn.Y = d_spd.Y;
  else Shift.Dyn.Y = Shift.Dyn.Y*(1.0f-r_spd) + d_spd.Y*r_spd;

  Vector2 alpha = (1.0f - Shift.Dyn / RegShift.Accuracy);
  
  alpha = alpha.Power(RegShift.Power);
  
  //RegShift.Test=alpha.X;
  
  Vector2 a_spd = alpha * RegSpd.Dynamic;
  
  //Vector2 s_spd = spd.Abs().Limit(RegSpd.Distance, 0.0f) / RegSpd.Distance;
  
  //a_spd += s_spd.Power(RegSpd.Power) * RegSpd.Alpha * alpha; // Это только для позиции
  
  a_spd=a_spd.Limit(1.0f, 0.0f);
  
  //a_spd=0.05f; // Убрать фильтр (раскоментить)
  
  RegShift.Test=a_spd.X;

  Shift.Spd  = spd * a_spd;
  //Shift.Pos = Shift.Spd * period;
  
  BufferPos += spd * period; // Восстановление позиции со сглаживанием (без потери точности)
  //Vector2 sub = BufferPos * BufferAlpha;
  //BufferPos-=sub;
  //Shift.Pos = sub;
  
  Shift.Pos = spd * BufferAlpha * period;
  
  ShiftSpd=a_spd;

  Shift.Prv = spd;

  Shift.Ready = true;
}

bool CameraFlow::GetShift(Vector2& Position, Vector2& Speed)
{
  bool ready = Shift.Ready;

  Position = Shift.Pos;
  Speed = Shift.Spd;

  if (ready) Shift.Prv -= Shift.Spd;

  Shift.Ready = false;

  return ready;
}

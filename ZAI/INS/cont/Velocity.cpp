#include <math.h>

#include "Velocity.h"

static constexpr float PI = 3.14159265359f;
static constexpr float TO_DEG = 180.0f / PI;
static constexpr float TO_RAD = PI / 180.0f;

static inline float sat(float val, float min, float max)
{
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

void Velocity::PointInLine(const Vector3& Current, Vector3& Point, float GroundShift)
{
  Vector3 end = PointEnd;
  
  if(GroundMode)
  {
    if (end.Z>GroundHeight) end.Z=GroundHeight;
  }
  
  Vector3 dir = end - PointBegin;
  Vector3 cur = PointBegin - Current;
  
  float len = dir.LengthSquared();
  float dis = cur.LengthSquared();
  
  if(len < (MinDistance * MinDistance))
    PointBegin = end;
  else if(dis < (MaxDistance * MaxDistance))
  {
    if(GroundMode) 
    {
      PointBegin += Vector2(dir).Norm(MoveSpeed*Period);
      if (dir.Z > 0.0f) PointBegin.Z += GroundSpeed*Period;
      else PointBegin.Z -= GroundSpeed*Period;
    }
    else PointBegin += dir.Norm(MoveSpeed*Period);
  }
  
  Point = PointBegin;
}

void Velocity::UpdateMoveSpeed(const Vector3& Position, const Vector3& Speed, const Vector3& Accel, float GroundShift)
{
  SpeedCurrent = Speed;
  
  Vector3 pos = Position;
  
  if (GroundMode) pos.Z -= GroundShift;
  
  Vector3 speed = Speed;//OriQuat.RotateAroundZ(Speed, true);

  static Vector3 auto_spd;
  if (ManualDirect) // Управление скоростью по направлению 
  {
    auto_spd = OriQuat.RotateAroundZ(SpeedXYZ);
  }
  else if (ManualSpeed) // Управление скоростью в глобальных координатах 
  {
    auto_spd = SpeedXYZ;
  }
  else // Управление скоростью по точкам
  {
    Vector3 point;
    PointInLine(pos, point, GroundShift);
    auto_spd = point - pos;
  }
  PointCurrent = pos;
  
  //auto_spd = OriQuat.RotateAroundZ(auto_spd, true);
  
  Vector2 xy
  {
    -RegSpeedY.Update(auto_spd.Y, speed.Y, -Accel.Y), // Pitch
    RegSpeedX.Update(auto_spd.X, speed.X, -Accel.X)   // Roll
  };
  
  float throt = RegSpeedT.Update(auto_spd.Z, speed.Z, -Accel.Z);  // Throt

  float pow = 0.5f + fabsf(throt);
  if (pow < 0.1f) pow = 0.1f;

  xy /= pow; // Компенсация вектора тяги
  xy.X = sat(xy.X, -1, 1);
  xy.Y = sat(xy.Y, -1, 1);

  xy.X = asinf(xy.X) * TO_DEG;
  xy.Y = asinf(xy.Y) * TO_DEG;

  if (xy.LengthSquared() > (AngleLimit * AngleLimit)) xy = xy.Norm(AngleLimit); // Ограничение наклона

  MoveXYZ = xy;
  MoveXYZ.Z = throt;
  
  //MoveXYZ = OriQuat.RotateAroundZ(MoveXYZ);

  if (ManualDirect) YawAngle = YawDirect;
  else if (AutoYaw) YawAngle = YawAuto;
  else YawAngle = YawManual;
}

void Velocity::LockAllRegInt(bool LockI)
{
  RegSpeedX.LockI = LockI;
  RegSpeedY.LockI = LockI;
  RegSpeedT.LockI = LockI;
}

void Velocity::SetManualYaw(bool Manual, const float* Yaw)
{
  AutoYaw = !Manual;
  if (Yaw) YawManual = *Yaw;
}

void Velocity::SetManualSpeed(bool Manual, const Vector3* XYZ)
{
  if (XYZ) SpeedXYZ = *XYZ;
  ManualSpeed = Manual;
}

void Velocity::SetDistanceSpeed(const Vector3* Begin, const Vector3* End, const float* NewSpeed)
{
  if (NewSpeed) MoveSpeed = fabsf(*NewSpeed);

  if (!End) return;
  PointEnd = *End;

  if (Begin) PointBegin = *Begin;
  /*else
  {
    if(StopSoft) PointBegin += SpeedCurrent * Period;
  }*/

  Vector2 xy = Vector2(PointEnd - PointBegin).Norm();
  float yaw = atan2f(xy.X, xy.Y) * TO_DEG;
  if (yaw < 0.0f) yaw = 360.0f + yaw;

  YawAuto = yaw;
}

void Velocity::SetDirectSpeedControl(bool DirectControl, const Vector3* XYZ, const float* Yaw)
{
  ManualDirect = DirectControl;

  if (!ManualDirect) return;

  if (XYZ) SpeedXYZ = *XYZ;
  if (Yaw) YawDirect = *Yaw;
}

void Velocity::Stop(const bool* SoftStop)
{
  bool stop = StopSoft;
  if (SoftStop) stop = *SoftStop;

  PointBegin = PointCurrent;
  if (stop) PointBegin += SpeedCurrent;
  PointEnd = PointBegin;
}

void Velocity::GetThrotAnglePRY(float* Throt, Vector3* AnglePRY, bool SpeedPRY[3])
{
  if (Throt) *Throt = MoveXYZ.Z;
  if (AnglePRY) *AnglePRY = { MoveXYZ.X, MoveXYZ.Y, YawAngle };
  if (SpeedPRY)
  {
    SpeedPRY[0] = false;
    SpeedPRY[1] = false;
    SpeedPRY[2] = ManualDirect;
  }
}

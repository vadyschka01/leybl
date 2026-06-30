#include <math.h>

#include "Attitude.h"

static constexpr float PI = 3.14159265359f;
static constexpr float TO_DEG = 180.0f / PI;
static constexpr float TO_RAD = PI / 180.0f;
static constexpr float G = 9.80665f; // m/c2

void Attitude::UpdateRegPRY(const Quaternion& CurQuat, const Vector3& SpeedPRY)
{
  Vector3 Gyr = SpeedPRY * TO_RAD;
  
  if (!SpeedMode)
  {
    static Quaternion error;
    error = CurQuat.GetError(GoalQuat, true);
    
    GoalSpeed.X = RegAngleP.Update(0.0f, -error.X, -Gyr.X);
    GoalSpeed.Y = RegAngleR.Update(0.0f, -error.Y, -Gyr.Y);
    GoalSpeed.Z = RegAngleY.Update(0.0f,  error.Z,  Gyr.Z);
  }
  
  Vector3 acc = (Gyr - LastSpeedPRY) / Period;
  
  PowerAngle.X = RegSpeedP.Update(GoalSpeed.X, Gyr.X, -acc.X);
  PowerAngle.Y = RegSpeedR.Update(GoalSpeed.Y, Gyr.Y, -acc.Y);
  PowerAngle.Z = RegSpeedY.Update(GoalSpeed.Z, -Gyr.Z, acc.Z);

  LastSpeedPRY = Gyr;
}

void Attitude::LockAllRegInt(bool LockI)
{
  RegAngleP.LockI = LockI;
  RegAngleR.LockI = LockI;

  RegSpeedP.LockI = LockI;
  RegSpeedR.LockI = LockI;
  RegSpeedY.LockI = LockI;
}

void Attitude::SetThrot(const float Throt)
{
  LastThrot = Throt;
}

void Attitude::SetSpeedPRY(const Vector3& SpeedTarget)
{
  GoalSpeed = SpeedTarget;
}

void Attitude::SetAnglePRY(const Quaternion& QuatTarget)
{
  GoalQuat = QuatTarget;
}

void Attitude::SetMode(const bool Speed)
{
  SpeedMode = Speed;
}

void Attitude::GetPowerPRY(float* Throt, Vector3* PowerPRY)
{
  if (Throt) *Throt = LastThrot;
  if (PowerPRY) *PowerPRY = PowerAngle;
}

#include <math.h>

#include "Attitude.h"

static constexpr float PI = 3.14159265359f;
static constexpr float TO_DEG = 180.0f / PI;
static constexpr float TO_RAD = PI / 180.0f;
static constexpr float G = 9.80665f; // m/c2

void Attitude::UpdateRegPRY(const Quaternion& CurQuat, const Vector3& SpeedPRY)
{
  if (!SpeedMode)
  {
    static Quaternion error;
    error = CurQuat.GetError(GoalQuat, true);
    
    GoalSpeed.X = RegAngleP.Update(0.0f, -error.X, -SpeedPRY.X);
    GoalSpeed.Y = RegAngleR.Update(0.0f, -error.Y, -SpeedPRY.Y);
    GoalSpeed.Z = RegAngleY.Update(0.0f, error.Z,  SpeedPRY.Z);
  }
  
  Vector3 Gyr = SpeedPRY * TO_RAD;
 
  
  Vector3 acc = (Gyr - LastSpeedPRY) / Period;
  
  //PowerAngle = GoalSpeed;
  
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

void Attitude::SetAnglePRY(const float* Throt, const Quaternion* QuatTarget, const bool* Speed)
{
  if (Throt) LastThrot = *Throt;

  if (QuatTarget) GoalQuat = *QuatTarget;

  if (Speed) SpeedMode = *Speed;
}

void Attitude::GetPowerPRY(float* Throt, Vector3* PowerPRY)
{
  if (Throt) *Throt = LastThrot;
  if (PowerPRY) *PowerPRY = PowerAngle;
}

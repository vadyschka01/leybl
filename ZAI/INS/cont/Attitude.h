#pragma once

#include "../geom/Vector.h"
#include "../util/Record.h"
#include "../geom/Quaternion.h"
#include "../util/RegPDI.h"

class Attitude
{
private:
  static constexpr float Freq = 100.0f;
  static constexpr float Period = 1.0f / Freq;

  bool SpeedMode = false;

  float LastThrot;

  Vector3 LastSpeedPRY;
  Vector3 PowerAngle;
  Vector3 GoalSpeed;
  Quaternion GoalQuat;

public:
  RegPDI RegAngleP, RegAngleR, RegAngleY;
  RegPDI RegSpeedP, RegSpeedR, RegSpeedY;

public:
  void UpdateRegPRY(const Quaternion& CurQuat, const Vector3& SpeedPRY); // Reversed не сделан !!!!!!!!!
  void LockAllRegInt(bool LockI);

  void SetAnglePRY(const float* Throt, const Quaternion* QuatTarget, const bool* SpeedPRY);

  void GetPowerPRY(float* Throt, Vector3* PowerPRY);
};

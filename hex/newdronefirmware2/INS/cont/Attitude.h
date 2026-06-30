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
  void UpdateRegPRY(const Quaternion& CurQuat, const Vector3& SpeedPRY); // Угол (stabilize)
  void LockAllRegInt(bool LockI);

  void SetThrot(const float Throt);
  void SetSpeedPRY(const Vector3& SpeedTarget);
  void SetAnglePRY(const Quaternion& QuatTarget);
  void SetMode(const bool Speed);

  void GetPowerPRY(float* Throt, Vector3* PowerPRY);
};

#pragma once

#include "../geom/Vector.h"
#include "../util/Record.h"
#include "../geom/Quaternion.h"
#include "../util/RegPDI.h"

class Velocity
{
public:
  Velocity(const Quaternion& quat) :OriQuat(quat) {}

  bool StopSoft = true; // true - учитывать тормозной путь при остановке и автовыборе начальной точки

  float MinDistance = 0.01f;
  float MaxDistance = 10.0f;
  float AngleLimit = 45.0f;
  
  bool GroundMode = false;
  float GroundSpeed = 1.0f;
  float GroundHeight = 5.0f;
  
  float TEST_SPEED;

private:
  static constexpr float Freq = 100.0f;
  static constexpr float Period = 1.0f / Freq;

  const Quaternion& OriQuat; // Главный кватернион ориентации из IRS

  Vector3 MoveXYZ;
  Vector3 SpeedCurrent;
  Vector3 PointCurrent;

  float YawAngle;

  float YawDirect;
  float YawManual;
  float YawAuto;
  bool AutoYaw = true;

  float MoveSpeed = 1.0f;

  Vector3 SpeedXYZ;
  bool ManualSpeed = false;
  bool ManualDirect = false;

private:
  void PointInLine(const Vector3& Current, Vector3& Point, float GroundShift);

public:
  Vector3 PointBegin;
  Vector3 PointEnd;
  RegPDI RegSpeedX, RegSpeedY, RegSpeedT;

public:
  void UpdateMoveSpeed(const Vector3& Position, const Vector3& Speed, const Vector3& Accel, float GroundShift);
  void LockAllRegInt(bool LockI);

  void SetManualYaw(bool Manual, const float* Yaw);
  void SetManualSpeed(bool Manual, const Vector3* XYZ);

  void SetDistanceSpeed(const Vector3* Begin, const Vector3* End, const float* NewSpeed);

  void SetDirectSpeedControl(bool DirectControl, const Vector3* XYZ, const float* Yaw); // упрввление скоростями сореинтированными на нос

  void Stop(const bool* SoftStop);

  void GetThrotAnglePRY(float* Throt, Vector3* AnglePRY, bool SpeedPRY[3]);
};

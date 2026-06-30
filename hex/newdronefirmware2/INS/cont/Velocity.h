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

  float AngleHalfSinLimit = 0.3827f; // sin(45/2)
  
  float MinDistance = 0.01f;
  float MaxDistance = 10.0f;
  float AngleLimit = 45.0f;
  
  bool GroundMode = false;
  float GroundSpeed = 3.0f; // Скорость высоты при GroundMode=true
  float GroundHeight = 3.0f; // Высота следования в режиме GroundMode=true
  float DistanceRabbit = 2.0f;
  float GroundLimitSettings = 1.0f; // Минимальная высота в режиме GroundMode=false (ее можно настроить через модуль оператора)
  float GroundLimit = GroundLimitSettings; // (эта меняется в завимисоти от полета)

private:
  static constexpr float Freq = 100.0f;
  static constexpr float Period = 1.0f / Freq;

  const Quaternion& OriQuat; // Главный кватернион ориентации из IRS

  Vector3 SpeedCurrent;
  Vector3 PointCurrent;

  Quaternion MoveQuat;
  float Throt;

  Vector2 YawManual = { 0.0f, 1.0f }; // half sin cos
  Vector2 YawAuto = { 0.0f, 1.0f };   // half sin cos
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
  void SetSpeed(const float* Speed3D, const float* GroundSpeed);

  void SetDistance(const Vector3* Begin, const Vector3& End);
  
  void SetAngleLimit(float AngleLimit);

  void Stop(const bool* SoftStop);

  void GetThrotAnglePRY(float& ThrotOut, Quaternion& QuatPRY);
};

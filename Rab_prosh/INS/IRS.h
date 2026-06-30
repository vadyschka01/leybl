#pragma once

#include "geom/Vector.h"
#include "geom/Quaternion.h"
#include "util/Record.h"

class IRS // Inertial Reference System
{
public:
  IRS();

  static constexpr float Freq = 100.0f;
  static constexpr float Period = 1.0f / Freq;

  Quaternion OriQuat; // Главный кватернион ориентации
  Vector3 OriPRT; // Наклоны SinX SinY CosZ локально ориентированные

  struct { Vector3 Gyr, Acc, Mag; } IMU; // Последние значения датчиков

  struct { Vector3 Acc, Spd, Pos; } Inertial; // Инерциальные значения движения
  
  float GroundShift = 0.0f; // Разница между высотой и поверхностью
  
  Quaternion ShiftAccelPRY; // Смещение акселерометра
  Vector2 NorthDeclination; // Смещение магнитометра
  
private:
  // Запись значений для расчётов в прошлом
  struct StructRecGyr
  {
    static constexpr float Past = 0.1f; // Максимальная запись прошлых данных
    static constexpr unsigned long Count = Freq * Past + 1;
    Vector3 Buffer[Count];
    Record<Vector3> Rec;
  } RecGyr;

  struct StructRecAccSpdPos
  {
    static constexpr float Past = 0.6f; // Максимальная запись прошлых данных
    static constexpr unsigned long Count = Freq * Past + 1;
    Vector3 Buffer[Count];
    Record<Vector3> Rec;
  } RecAcc, RecSpd, RecPos;

  struct StructRecQua
  {
    static constexpr float Past = 0.6f; // Максимальная запись прошлых данных
    static constexpr unsigned long Count = Freq * Past + 1;
    Quaternion Buffer[Count];
    Record<Quaternion> Rec;
  } RecQua;
  //---

  // Сдвиг данных
  struct { Vector3 Pos, Spd; Quaternion Qua = 0.0f; } Shift; // Значение смещения позиции, скорости и кватерниона
  //---

public:
  Record<Vector3>& RecordGyro;
  Record<Vector3>& RecordSpeed;
  Record<Vector3>& RecordPosit;
  Record<Quaternion>& RecordQuat;

private:
  void RestoreQuat(const Vector3& Acc);
  static float GetAngle(float X, float Y, float Z);

public:
  void UpdateGyro(const Vector3& Gyr); // deg/s
  void UpdateMagnet(const Vector3& Mag); // gaus
  void UpdateAccel(const Vector3& Acc); // g

  void SetShiftAlpha(const Vector3* Pos, const Vector3* Spd, const float* Qua); // Коэффициенты восстановления позиции и скорости в секунду

  void UpdatePositionSpeed(const Vector3& ShiftPosition, const Vector3& ShiftSpeed);
  void UpdateQuaternion(const Quaternion& ShiftQuaternion, float Alpha);

  void RestoreAllShift(Vector3& ShiftPos);
  
  void SetAccelShift(float Pitch, float Roll, float Yaw);
  void SetNorthDeclination(const float Yaw);
  
  bool SetGroundHeight(float Height, float Alpha);

public:
  void GetSinXYCosZ(Vector3& SinXYCos);
  void GetPitchRollYaw(Vector3& PitchRollYaw, bool& Reversed);
  void GetInertial(Vector3* Pos, Vector3* Spd, Vector3* Acc, float* Gnd);
};
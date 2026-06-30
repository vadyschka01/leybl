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

Vector2 TargetOnXY(const Vector2& Point, const Vector2& Start, const Vector2& End, const float Distance) 
{
  Vector2 line = End - Start;
  float len = line.LengthSquared();
  if (len < 1e-6f) return End;
  
  Vector2 dis = Point - Start;
  
  float t = dis.Dot(line) / len;
  t = sat(t, 0.0f, 1.0f);
  
  Vector2 proj = Start + line * t;
  
  float way = (End - proj).LengthSquared();
  
  Vector2 target = line.Norm(Distance);
  
  if (way < target.LengthSquared()) return End;
  
  return proj + target;
}

Vector2 TargetSpeed2D(const Vector3& Point, const Vector3& Start, const Vector3& End, const float Distance, const float Speed)
{
  Vector2 line = End - Point;
  
  float speed = sat(line.Length(), 0.0f, Speed);
  
  Vector2 target = TargetOnXY(Point, Start, End, Distance);
  
  Vector2 move_xy = (target - Point).Norm(speed);
  
  return move_xy;
}

Vector3 TargetSpeed3D(const Vector3& Point, const Vector3& Start, const Vector3& End, const float Distance, const float Speed)
{
    Vector3 line = End - Start;
    float len = line.LengthSquared();
    
    float distToEnd = (End - Point).Length();
    float speed = sat(distToEnd, 0.0f, Speed);

    // Если путь очень короткий, просто летим к конечной точке
    if (len < 1e-6f) return (End - Point).Norm(speed);

    // Находим проекцию на 3D-линию
    Vector3 dis = Point - Start;
    float t = dis.Dot(line) / len;
    t = sat(t, 0.0f, 1.0f); // Ограничиваем проекцию границами отрезка
    
    Vector3 proj = Start + line * t; // Ближайшая точка на 3D-отрезке

    Vector3 dir = line.Norm(); // Единичный вектор направления
    Vector3 target = proj + dir * Distance;

    // Проверяем, не вышли ли мы за конечную точку
    float way = (End - proj).LengthSquared();
    if (way < (Distance * Distance)) target = End;
    
    // Формируем единый 3D-вектор скорости // Направлен от текущей позиции к 3D-цели
    return (target - Point).Norm(speed);
}

void Velocity::UpdateMoveSpeed(const Vector3& Position, const Vector3& Speed, const Vector3& Accel, float GroundShift)
{
  SpeedCurrent = Speed;
  Vector3 pos = Position;
  
  Vector3 speed = Speed;

  static Vector3 auto_spd;


  Vector3 start=PointBegin, end=PointEnd;
  
  if (GroundMode)
  {
    float heigth = Position.Z - GroundShift;
    
    auto_spd = TargetSpeed2D(Position, start, end, DistanceRabbit, MoveSpeed);
    auto_spd.Z = sat((GroundHeight - heigth), -GroundSpeed, GroundSpeed);
  }
  else
  {
    float heigth = GroundLimit+GroundShift;
    
    if (start.Z < heigth) start.Z = heigth;
    if (end.Z < heigth) end.Z = heigth;
    
    auto_spd = TargetSpeed3D(Position, start, end, DistanceRabbit, MoveSpeed);
  }
  
  if(TransitionTimer > Period)
  {
    float ratio = Period / TransitionTimer; 
    AutoSpeed = AutoSpeed + (auto_spd - AutoSpeed)*ratio;
    TransitionTimer-=Period;
  }
  else AutoSpeed=auto_spd;
  
  Vector2 xy // half sin x & y
  {
    -RegSpeedY.Update(AutoSpeed.Y, speed.Y, -Accel.Y), // Pitch
    RegSpeedX.Update(AutoSpeed.X, speed.X, -Accel.X)   // Roll
  };
  
  Throt = RegSpeedT.Update(AutoSpeed.Z, speed.Z, -Accel.Z);  // Throt

  if (xy.LengthSquared() > (AngleHalfSinLimit * AngleHalfSinLimit)) xy = xy.Norm(AngleHalfSinLimit); // Ограничение наклона
  
  Vector3 xyz = xy;
  xyz.Z = AutoYaw ? YawAuto.X : YawManual.X;
  
  float c = AutoYaw ? YawAuto.Y : YawManual.Y;
  
  Quaternion desired = Quaternion::CreateYawPitchRoll(xyz, 0, &c);
  
  MoveQuat = desired;
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
  if (Yaw) YawManual = { sinf(*Yaw * TO_RAD * 0.5f), cosf(*Yaw * TO_RAD * 0.5f)};
}

void Velocity::SetSpeed(const float* Speed3D, const float* SpeedGround)
{
  if(Speed3D) MoveSpeed=fabsf(*Speed3D);
  if(SpeedGround) GroundSpeed=fabsf(*SpeedGround);
}

void Velocity::SetDistance(const Vector3* Begin, const Vector3& End, float TransitionTime)
{
  if(TransitionTime>10.0f) TransitionTime=10.0f;
  TransitionTimer=TransitionTime;
  
  PointEnd = End;

  if (Begin) PointBegin = *Begin;

  float s, c;
  YawAuto = (PointEnd - PointBegin).GetDirHalfSinCos();
}

void Velocity::SetAngleLimit(float AngleLimit)
{
  AngleHalfSinLimit = sinf(AngleLimit * 0.5f);
}

void Velocity::Stop(const bool* SoftStop)
{
  bool stop = StopSoft;
  if (SoftStop) stop = *SoftStop;

  PointBegin = PointCurrent;
  if (stop) PointBegin += SpeedCurrent;
  PointEnd = PointBegin;
}

void Velocity::GetThrotAnglePRY(float& ThrotOut, Quaternion& QuatPRY)
{
  ThrotOut = Throt;
  QuatPRY = MoveQuat;
}

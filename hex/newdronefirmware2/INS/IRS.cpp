#include <math.h>

#include "IRS.h"

static constexpr float PI = 3.14159265359f;
static constexpr float TO_DEG = 180.0f / PI;
static constexpr float TO_RAD = PI / 180.0f;
static constexpr float G = 9.80665f; // m/c2

IRS::IRS():
  RecordGyro(RecGyr.Rec),
  RecordSpeed(RecSpd.Rec),
  RecordPosit(RecPos.Rec),
  RecordQuat(RecQua.Rec)
{
  RecordGyro.Init(RecGyr.Buffer, Freq, RecGyr.Past);
  RecordSpeed.Init(RecSpd.Buffer, Freq, RecSpd.Past);
  RecordPosit.Init(RecPos.Buffer, Freq, RecPos.Past);
  RecordQuat.Init(RecQua.Buffer, Freq, RecQua.Past);
}

void IRS::RestoreQuat(const Vector3& Acc)
{
  float acc_alpha=0.1f;
  static Vector3 raw;
    
  raw = raw * (1.0f-acc_alpha) + Acc * acc_alpha;
  
  float len = raw.Length();

  static float quat_acc_alpha = 0.03f;
  static float quat_acc_max = 0.02f;
  
  float dyn = fabsf(len - 1.0f);
  if(dyn>quat_acc_max) dyn=1.0f; else dyn/=quat_acc_max;

  static float gain = 0.0f;
  
  gain = quat_acc_alpha * (1.0f - dyn);
    
  if (gain < 0.001f) return;
  
  Vector3 acc = Acc.Norm();
  
  Vector3 est;
  est.X = 2.0f * (OriQuat.X * OriQuat.Z - OriQuat.W * OriQuat.Y);
  est.Y = 2.0f * (OriQuat.W * OriQuat.X + OriQuat.Y * OriQuat.Z);
  est.Z = OriQuat.W * OriQuat.W - OriQuat.X * OriQuat.X - OriQuat.Y * OriQuat.Y + OriQuat.Z * OriQuat.Z;

  Vector3 cross = acc.Cross(est); 
  float dot = acc.Dot(est);
  
  if (dot < 0.0f) 
  {
    float error_len = cross.Length();
    if (error_len < 0.001f) cross = {1.0f, 0.0f, 0.0f}; 
    else cross = cross * (1.0f / error_len);
  }
  
  Vector3 axis = cross * (gain * 0.5f);
  
  Quaternion correction 
  {
    axis.X,
    axis.Y,
    axis.Z,
    1.0f // Для малых углов cos(0) = 1. При нормализации это сработает корректно.
  };

  OriQuat = OriQuat * correction;
  
  OriQuat = OriQuat.Norm();
}

float IRS::GetAngle(float X, float Y, float Z)
{
  if (Y == 0.0f && Z == 0.0f)
  {
    if (X > 0.0f) return 90.0f;
    if (X < 0.0f) return -90.0f;
    return 0.0f;
  }

  return atanf(X / sqrtf(Y * Y + Z * Z)) * TO_DEG;
}

void IRS::UpdateGyro(const Vector3& Gyr)
{
  if(!Gyr.IsFinite()) return;
  
  IMU.Gyr = Gyr;

  Vector3 gyr = Gyr * (Period * TO_RAD);

  RecordGyro.Add(Gyr * TO_RAD);

  Quaternion g = gyr;
  g = (OriQuat * g);

  OriQuat += g * 0.5f;
  OriQuat = OriQuat.Norm();

  RecordQuat.Add(OriQuat);

  OriPRT = OriQuat.Conjugate() * Quaternion(0, 0, 1, 0) * OriQuat;
}

void IRS::UpdateMagnet(const Vector3& Mag)
{
  if(!Mag.IsFinite()) return;
  
  IMU.Mag = Mag; // Сохраняем сырые данные (если нужно для отладки)
  
  static int init_count = 100;
  
  float alpha = 0.001f; 
  

  if(init_count > 0) 
  {
    alpha = 0.05f;
    init_count--;
  }

  Vector3 mag_norm = Mag.Norm();
  
  Quaternion mW = OriQuat * mag_norm * OriQuat.Conjugate();
  
  Vector3 mW_horizontal = { mW.X, mW.Y, 0.0f };
  
  if (mW_horizontal.LengthSquared() < 0.0001f) return;
  mW_horizontal = mW_horizontal.Norm();

  float error_z = mW_horizontal.Y * NorthDeclination.X - mW_horizontal.X * NorthDeclination.Y;
  
  float half_error_step = error_z * alpha * 0.5f;
  
  Quaternion corr
  {
      0.0f,
      0.0f,
      half_error_step, 
      1.0f 
  };
  
  OriQuat = corr * OriQuat;
  OriQuat = OriQuat.Norm();
}

void IRS::UpdateAccel(const Vector3& Acc)
{
  if(!Acc.IsFinite()) return;
  
  Vector3 accel = ShiftAccelPRY * Acc * ShiftAccelPRY.Conjugate();
  
  IMU.Acc = accel;

  Quaternion ine = OriQuat * accel * OriQuat.Conjugate();

  Vector3 acc = { ine.X, ine.Y, ine.Z - 1.0f }; // без гравитации
  
  Inertial.Acc = acc;//OriQuat.RotateAroundZ(acc, true);

  Inertial.Spd += acc * (G * Period); // интегрирование скорости
  
  Vector3 spd = Inertial.Spd * Period;
  
  Inertial.Pos += spd; // интегрирование позиции
  
  RecordSpeed.Add(Inertial.Spd);
  RecordPosit.Add(Inertial.Pos);

  RestoreQuat(accel);
}

void IRS::UpdatePositionSpeed(const Vector3& ShiftPosition, const Vector3& ShiftSpeed)
{
  Shift.Pos += ShiftPosition;
  Shift.Spd += ShiftSpeed;
}

void IRS::UpdateQuaternion(const Quaternion& ShiftQuaternion, float Alpha)
{
  Shift.Pos = Shift.Qua * (1.0f - Alpha) + ShiftQuaternion * Alpha;
}

void IRS::RestoreAllShift(Vector3& ShiftPos)
{
  Vector3 spd = Shift.Spd;
  Shift.Spd = 0;
  RecordSpeed.Mix(spd);
  Inertial.Spd += spd;

  Vector3 pos = Shift.Pos;
  Shift.Pos = 0;
  
  RecordPosit.Mix(pos);
  Inertial.Pos += pos;
  ShiftPos=pos;

  Quaternion qua = Shift.Qua;
  Shift.Qua = 0;
  RecordQuat.Mix(qua);
  OriQuat += qua;
  OriQuat = OriQuat.Norm();
}

void IRS::SetAccelShift(float Pitch, float Roll, float Yaw)
{
  float h_roll  = (Roll * TO_RAD)  / 2.0f;
  float h_pitch = (Pitch * TO_RAD) / 2.0f;
  float h_yaw   = (Yaw * TO_RAD)   / 2.0f;
  
  Quaternion q_roll =  {0.0f, sinf(h_roll), 0.0f, cosf(h_roll)};
  Quaternion q_pitch = {sinf(h_pitch), 0.0f, 0.0f, cosf(h_pitch)};
  Quaternion q_yaw = {0.0f, 0.0f, sinf(h_yaw), cosf(h_yaw)}; //   Как найти поворот: arcsin(ось/макс_ось) * TO_GRAD
  
  ShiftAccelPRY = q_pitch * q_roll * q_yaw;
}

void IRS::SetNorthDeclination(const float Yaw)
{
  float dec = Yaw * TO_RAD; // Укажите ваше склонение
  Vector3 mag_north_target;
  NorthDeclination = { sinf(dec), cosf(dec) };
}

bool IRS::SetGroundHeight(float Height, float Alpha)
{
  float shift = Inertial.Pos.Z - Height;
  GroundShift = GroundShift*(1.0f-Alpha) + shift*Alpha;
  
  float height = Inertial.Pos.Z - GroundShift;
  if (height < 0.0f) GroundShift+=height;
}

/*bool IRS::FixedHeight(bool Fixed, float Height)
{
  bool zero=false;
  
  if(Fixed)
  { 
    if (Inertial.Pos.Z < Height) 
    {
      FixedMinHeight = Inertial.Pos.Z;
      zero = true;
    }
    else FixedMinHeight = Height;
  }
  
  if (!Fixed || FixedMinHeight < 0.0f) FixedMinHeight = 0.0f;
  
  return zero;
}*/

void IRS::GetSinXYCosZ(Vector3& SinXYCosZ)
{
  SinXYCosZ = OriPRT;
}

void IRS::GetPitchRollYaw(Vector3& PitchRollYaw, bool& Reversed)
{
  Reversed = OriPRT.Z < 0;
  
  float yaw = atan2f(2.0f * (OriQuat.X * OriQuat.Y - OriQuat.W * OriQuat.Z), 1.0f - 2.0f * (OriQuat.Y * OriQuat.Y + OriQuat.Z * OriQuat.Z)) * TO_DEG;
  if (yaw < 0.0f) yaw += 360.0f;

  PitchRollYaw =
  {
    GetAngle(OriPRT.Y, OriPRT.X, OriPRT.Z),  // Pitch
    GetAngle(-OriPRT.X, OriPRT.Y, OriPRT.Z), // Roll
    yaw                                      // Yaw
  };
}

void IRS::GetInertial(Vector3* Pos, Vector3* Spd, Vector3* Acc, float* Gnd)
{
  if (Acc) *Acc = Inertial.Acc;
  if (Spd) *Spd = Inertial.Spd;
  if (Pos) *Pos = Inertial.Pos;
  if (Gnd) *Gnd = GroundShift;
}

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
  float len = Acc.Length();

  static float quat_acc_alpha = 0.03f;
  static float quat_acc_max = 0.02f;
  
  float dyn = fabsf(len - 1.0f);
  if(dyn>quat_acc_max) dyn=1.0f; else dyn/=quat_acc_max;

  float gain = quat_acc_alpha * (1.0f - dyn);
  
  if (gain < 0.0001f) return;
  
  Vector3 acc = Acc.Norm();
  //acc.Z=-acc.Z;
  
  Vector3 est;
  est.X = 2.0f * (OriQuat.X * OriQuat.Z - OriQuat.W * OriQuat.Y);
  est.Y = 2.0f * (OriQuat.W * OriQuat.X + OriQuat.Y * OriQuat.Z);
  est.Z = OriQuat.W * OriQuat.W - OriQuat.X * OriQuat.X - OriQuat.Y * OriQuat.Y + OriQuat.Z * OriQuat.Z;
  
  Vector3 cross = acc.Cross(est);

  float dot = est.X * acc.X + est.Y * acc.Y + est.Z * acc.Z;

  Quaternion correction = { cross.X * gain, cross.Y * gain, cross.Z * gain, 1.0f + dot * gain };
  correction = correction.Norm();
  
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
  IMU.Mag = Mag;
  
  static int init_count=100;
  
  float alpha = 0.0001f;
  
  if(init_count>0) 
  {
    alpha=0.05f;
    init_count--;
  }
  
  Vector3 mag = Mag.Norm();
  
  Quaternion mW = OriQuat * mag * OriQuat.Conjugate();
  
  static float declination = -7.0f;
  
  while(declination >  180.0f) declination-=360.0f;
  while(declination < -180.0f) declination+=360.0f;
  
  Quaternion noth 
  {
      0.0f,             // x
      0.0f,             // y
      sinf(declination * TO_RAD * 0.5f), // z
      cosf(declination * TO_RAD * 0.5f)  // w
  };
  
  mW = noth * mW;
  
  float heading_error = -atan2f(mW.X, mW.Y);
  
  float half_angle = (alpha * heading_error) * 0.5f;
  
  Quaternion corr 
  {
      0.0f,             // x
      0.0f,             // y
      sinf(half_angle), // z
      cosf(half_angle)  // w
  };
  
  if(corr.IsNAN()) return;
  
  OriQuat = corr * OriQuat;
  
  OriQuat = OriQuat.Norm();
}

void IRS::UpdateAccel(const Vector3& Acc)
{
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

bool IRS::SetGroundHeight(float* Height, float MaxHeight, float Alpha)
{
  if(Height)
  {
    float shift = Inertial.Pos.Z - *Height;
    GroundShift = GroundShift*(1.0f-Alpha) + shift*Alpha;
  }
  else
  {
    float height = Inertial.Pos.Z - GroundShift;
    
    if(height<MaxHeight)
    {
      float shift = Inertial.Pos.Z - MaxHeight;
      GroundShift = GroundShift*(1.0f-Alpha) + shift*Alpha;
    }
  }
  
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

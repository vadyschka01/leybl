#include <math.h>

#include "Quaternion.h"

#define fast_cos(s2) (1.0f - s2 * (0.5f + 0.125f * s2))

void Quaternion::Zero()
{
  X = 0.0f; Y = 0.0f; Z = 0.0f; W = 1.0f;
}

Quaternion Quaternion::Norm(float Gain) const
{
  float norm = sqrtf(X * X + Y * Y + Z * Z + W * W);

  if (norm > 1e-6f)
  {
    norm = Gain / norm;

    return
    {
      X * norm,
      Y * norm,
      Z * norm,
      W * norm,
    };
  }
  
  return { 0.0f, 0.0f, 0.0f, 0.0f };
}

Quaternion Quaternion::Conjugate() const
{
  return { -X, -Y, -Z, W };
}

Quaternion Quaternion::Invert() const
{
  float nsq = X * X + Y * Y + Z * Z + W * W;

  if (nsq > 1e-6f)
  {
    nsq = 1.0f / nsq;

    return
    {
      -X * nsq,
      -Y * nsq,
      -Z * nsq,
      W * nsq,
    };
  }

  return { 0.0f, 0.0f, 0.0f, 0.0f };
}

Quaternion Quaternion::Negate() const
{
  return { -X, -Y, -Z, -W };
}

bool Quaternion::IsNAN() const
{
  return (X != X) || (Y != Y) || (Z != Z) || (W != W);
}

Quaternion& Quaternion::operator=(const Quaternion& Q)
{
  W = Q.W;
  X = Q.X;
  Y = Q.Y;
  Z = Q.Z;

  return *this;
}

Quaternion& Quaternion::operator+=(const Quaternion& Q)
{
  X += Q.X;
  Y += Q.Y;
  Z += Q.Z;
  W += Q.W;

  return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& Q)
{
  X -= Q.X;
  Y -= Q.Y;
  Z -= Q.Z;
  W -= Q.W;

  return *this;
}

Quaternion& Quaternion::operator*=(const float Value)
{
  X *= Value;
  Y *= Value;
  Z *= Value;
  W *= Value;

  return *this;
}

Quaternion& Quaternion::operator*=(const Quaternion& Q)
{
  const float x = X;
  const float y = Y;
  const float z = Z;
  const float w = W;
  
  X = w * Q.X + x * Q.W + y * Q.Z - z * Q.Y;
  Y = w * Q.Y - x * Q.Z + y * Q.W + z * Q.X;
  Z = w * Q.Z + x * Q.Y - y * Q.X + z * Q.W;
  W = w * Q.W - x * Q.X - y * Q.Y - z * Q.Z;

  return *this;
}

Quaternion Quaternion::operator*(const float Value) const
{
  return
  {
    X * Value,
    Y * Value,
    Z * Value,
    W * Value,
  };
}

Quaternion Quaternion::operator*(const Quaternion& Q) const
{
  return
  {
    W * Q.X + X * Q.W + Y * Q.Z - Z * Q.Y,
    W * Q.Y - X * Q.Z + Y * Q.W + Z * Q.X,
    W * Q.Z + X * Q.Y - Y * Q.X + Z * Q.W,
    W * Q.W - X * Q.X - Y * Q.Y - Z * Q.Z,
  };
}

Quaternion Quaternion::operator+(const Quaternion& Q) const
{
  return
  {
    X + Q.X,
    Y + Q.Y,
    Z + Q.Z,
    W + Q.W,
  };
}

Quaternion Quaternion::operator-(const Quaternion& Q) const
{
  return
  {
    X - Q.X,
    Y - Q.Y,
    Z - Q.Z,
    W - Q.W,
  };
}

Vector3 Quaternion::Rotate(const Vector3& vec) const
{
  Quaternion p = { vec.X, vec.Y, vec.Z, 0.0f };
  
  // Вычисляем p' = q * p * q^ (q^ - сопряженный)
  Quaternion rotated = *this * p * this->Conjugate();
  
  // Возвращаем векторную часть результата
  return { rotated.X, rotated.Y, rotated.Z };
}

Vector3 Quaternion::RotateAroundZ(const Vector3& vec, bool CCW) const
{
  float yaw_sin_term = 2.0f * (W * Z + X * Y);
  float yaw_cos_term = 1.0f - 2.0f * (Y * Y + Z * Z);
  
  float mag_sq = yaw_sin_term * yaw_sin_term + yaw_cos_term * yaw_cos_term;
  
  if (mag_sq < 1e-6f) return vec;
  
  float inv_mag = 1.0f / sqrtf(mag_sq); 
  
  float c = yaw_cos_term * inv_mag;
  float s = yaw_sin_term * inv_mag;

  if (CCW) s = -s;

  return
  {
    vec.X * c - vec.Y * s,
    vec.X * s + vec.Y * c,
    vec.Z
  };
}

Quaternion Quaternion::CreateYawPitchRoll(const Vector3& PitchRollYawHalfSin, const Vector3* PitchRollYawHalfCos, const float* YawHalfCos) // Глобальный sin/2
{
  float sp =  PitchRollYawHalfSin.X, cp;
  float sr =  PitchRollYawHalfSin.Y, cr;
  float sy = -PitchRollYawHalfSin.Z, cy;
  
  if(PitchRollYawHalfCos)
  {
    cp = PitchRollYawHalfCos->X;
    cr = PitchRollYawHalfCos->Y;
    cy = PitchRollYawHalfCos->Z;
  }
  else
  {
    cp = fast_cos(sp * sp);
    cr = fast_cos(sr * sr);
    if(YawHalfCos) cy = *YawHalfCos;
    else cy = fast_cos(sy * sy);
  }
  
  return // Это эквивалент q_roll(Y) * q_pitch(X) * q_yaw(Z) [Yaw -> Pitch -> Roll]
  {
    cy * cr * sp + sy * sr * cp, // X
    cy * sr * cp - sy * cr * sp, // Y
    sy * cr * cp + cy * sr * sp, // Z
    cy * cr * cp - sy * sr * sp  // W
  };
}

Quaternion Quaternion::CreateYawPitchRoll(const Vector3& PitchRollYawRad) // Глобальный
{
  float hp =  0.5f * PitchRollYawRad.X;
  float hr =  0.5f * PitchRollYawRad.Y;
  float hy = -0.5f * PitchRollYawRad.Z;

  float sp = sinf(hp), cp = cosf(hp);
  float sr = sinf(hr), cr = cosf(hr);
  float sy = sinf(hy), cy = cosf(hy);
  
  return // Это эквивалент q_roll(Y) * q_pitch(X) * q_yaw(Z) [Yaw -> Pitch -> Roll]
  {
    cy * cr * sp + sy * sr * cp, // X
    cy * sr * cp - sy * cr * sp, // Y
    sy * cr * cp + cy * sr * sp, // Z
    cy * cr * cp - sy * sr * sp  // W
  };
}

Quaternion Quaternion::CreatePitchRollYaw(const Vector3& PitchRollYawHalfSin, const Vector3* PitchRollYawHalfCos, const float* YawHalfCos) // Локальный sin/2
{
  float sp =  PitchRollYawHalfSin.X, cp;
  float sr =  PitchRollYawHalfSin.Y, cr;
  float sy = -PitchRollYawHalfSin.Z, cy;
  
  if(PitchRollYawHalfCos)
  {
    cp = PitchRollYawHalfCos->X;
    cr = PitchRollYawHalfCos->Y;
    cy = PitchRollYawHalfCos->Z;
  }
  else
  {
    cp = fast_cos(sp * sp);
    cr = fast_cos(sr * sr);
    if(YawHalfCos) cy = *YawHalfCos;
    else cy = fast_cos(sy * sy);
  }
  
  return // Эквивалент q_yaw(Z) * q_pitch(X) * q_roll(Y)  [ Roll -> Pitch -> Yaw]
  {
    cy * cr * sp - sy * sr * cp, // X
    cy * sr * cp + sy * cr * sp, // Y
    sy * cr * cp - cy * sr * sp, // Z
    cy * cr * cp + sy * sr * sp  // W
  };
}

Quaternion Quaternion::CreatePitchRollYaw(const Vector3& PitchRollYawRad) // Локальный
{
  float hp =  0.5f * PitchRollYawRad.X;
  float hr =  0.5f * PitchRollYawRad.Y;
  float hy = -0.5f * PitchRollYawRad.Z;

  float sp = sinf(hp), cp = cosf(hp);
  float sr = sinf(hr), cr = cosf(hr);
  float sy = sinf(hy), cy = cosf(hy);
  
  return // Эквивалент q_yaw(Z) * q_pitch(X) * q_roll(Y)  [ Roll -> Pitch -> Yaw]
  {
    cy * cr * sp - sy * sr * cp, // X
    cy * sr * cp + sy * cr * sp, // Y
    sy * cr * cp - cy * sr * sp, // Z
    cy * cr * cp + sy * sr * sp  // W
  };
}

Quaternion Quaternion::CreateYaw(const float YawRad)
{
  float hy = - 0.5f * YawRad;
  return { 0.0f, 0.0f, sinf(hy), cosf(hy) };
}

Quaternion Quaternion::GetError(const Quaternion& Target, bool FastWay) const
{
  Quaternion error // Формула произведения Гамильтона с учетом инверсии current
  {
    W * Target.X - X * Target.W - Y * Target.Z + Z * Target.Y,
    W * Target.Y + X * Target.Z - Y * Target.W - Z * Target.X,
    W * Target.Z - X * Target.Y + Y * Target.X - Z * Target.W,
    W * Target.W + X * Target.X + Y * Target.Y + Z * Target.Z
  };
  
  if (FastWay && (error.W < 0.0f)) return error.Negate();
  
  return error;
}

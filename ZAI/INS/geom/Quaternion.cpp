#include <math.h>

#include "Quaternion.h"

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
  
  return *this;
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
      X * nsq,
      Y * nsq,
      Z * nsq,
      W * nsq,
    };
  }

  return *this;
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
  // 1. Извлекаем "сырые" компоненты синуса и косинуса угла рыскания (Yaw)
  // Это алгебраическая проекция кватерниона на плоскость вращения Z.
  // Формула соответствует: yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
  
  float yaw_sin_term = 2.0f * (W * Z + X * Y);
  float yaw_cos_term = 1.0f - 2.0f * (Y * Y + Z * Z);
  // 2. Нормализация вектора (sin, cos)
  // Это заменяет вычисление угла и обратный sin/cos.
  // Мы превращаем (yaw_cos_term, yaw_sin_term) в единичный вектор направления.
  
  float mag_sq = yaw_sin_term * yaw_sin_term + yaw_cos_term * yaw_cos_term;
  
  // ЗАЩИТА ОТ GIMBAL LOCK (Singularity)
  // Если дрон вертикально вверх/вниз (Pitch ~90), Yaw теряет смысл.
  // В этом случае считаем поворот нулевым (cos=1, sin=0).
  if (mag_sq < 1e-6f) 
  {
      // Возвращаем вектор без вращения, чтобы не получить NaN
      return vec; 
  }
  
  // Быстрая нормализация (можно использовать FastInvSqrt если есть в либе)
  float inv_mag = 1.0f / sqrtf(mag_sq); 
  
  float c = yaw_cos_term * inv_mag;
  float s = yaw_sin_term * inv_mag;
  // 3. Учет направления вращения
  if (CCW) s = -s;
  // 4. Вращение вектора (Матрица поворота 2D)
  // Z остается неизменным, так как вращаем ВОКРУГ Z.
  return Vector3(
    vec.X * c - vec.Y * s,
    vec.X * s + vec.Y * c,
    vec.Z
  );
}

/*Vector3 Quaternion::RotateAroundZ(const Vector3& vec, bool CCW) const
{
  Quaternion v = { vec.X, vec.Y, 0, 0 };
  Quaternion q = { 0, 0, CCW ? Z : -Z, W };
  q = q.Norm();

  q = (v * q) * q;

  return { q.X, q.Y, vec.Z };
}*/

Quaternion Quaternion::CreateYawPitchRoll(const Vector3& PitchRollYawRad)
{
  float hp = 0.5f * PitchRollYawRad.X;
  float hr = 0.5f * PitchRollYawRad.Y;
  float hy = 0.5f * PitchRollYawRad.Z;

  float cr = cosf(hr), sr = sinf(hr);
  float cp = cosf(hp), sp = sinf(hp);
  float cy = cosf(hy), sy = sinf(hy);
  
  return // Это эквивалент q_roll(Y) * q_pitch(X) * q_yaw(Z) [Yaw -> Pitch -> Roll]
  {
    cr * sp * cy - sr * cp * sy,
    sr * cp * cy + cr * sp * sy,
   -cr * cp * sy - sr * sp * cy,
    cr * cp * cy - sr * sp * sy
  };
}

Quaternion Quaternion::CreateYaw(const float YawRad)
{
  float h_y = 0.5f * YawRad;
  return { 0.0f, 0.0f, sinf(h_y), cosf(h_y) };
}

Quaternion Quaternion::CreateDirection(const Vector2& Course)
{
  Vector2 xy = Course.Norm();
  
  if(xy.X < -0.999f) return { 0.0, 0.0, 1.0, 0.0 }; // Поворот на 180 градусов
  float w = sqrtf((1.0f + xy.X) * 0.5f);
  return { 0.0f, 0.0f, xy.Y / (2.0f * w), w };
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

#include <math.h>
#include "Quaternion.h"

#include "Vector.h"

Vector2::Vector2(const Vector3& Vec) : X(Vec.X), Y(Vec.Y) {}

void Vector2::Zero()
{
  X = Y = 0;
}

Vector2 Vector2::Norm(float Gain) const
{
  float n = sqrtf(X * X + Y * Y);

  if (n > 1e-12f)
  {
    n = Gain / n;

    return
    {
      X * n,
      Y * n
    };
  }

  return {0, 0};
}

Vector2 Vector2::Abs() const
{
  return { fabsf(X) , fabsf(Y) };
}

float Vector2::Length() const
{
  return sqrtf(X * X + Y * Y);
}

float Vector2::LengthSquared() const
{
  return X * X + Y * Y;
}

bool Vector2::IsNAN() const
{
  return std::isnan(X) || std::isnan(Y);
}

bool Vector2::IsFinite() const 
{
  return (X - X == 0) && (Y - Y == 0);
}

Vector2 Vector2::Limit(float Max, float Min) const
{
  Vector2 lim;
  
  if (X > Max) lim.X = Max; else if (X < Min) lim.X = Min; else lim.X = X;
  if (Y > Max) lim.Y = Max; else if (Y < Min) lim.Y = Min; else lim.Y = Y;
  
  return lim;
}

Vector2 Vector2::Power(float Pow) const
{
  return
  {
    powf(X, Pow),
    powf(Y, Pow)
  };
}

float Vector2::Dot(const Vector2& Vec) const
{
  return X * Vec.X + Y * Vec.Y;
}

Vector2& Vector2::operator+=(const Vector2& Vec)
{
  X += Vec.X;
  Y += Vec.Y;

  return *this;
}

Vector2& Vector2::operator-=(const Vector2& Vec)
{
  X -= Vec.X;
  Y -= Vec.Y;

  return *this;
}

Vector2& Vector2::operator*=(float Val)
{
  X *= Val;
  Y *= Val;

  return *this;
}

Vector2& Vector2::operator*=(const Vector2& Vec)
{
  X *= Vec.X;
  Y *= Vec.Y;

  return *this;
}

Vector2& Vector2::operator/=(float Val)
{
  X /= Val;
  Y /= Val;

  return *this;
}

Vector2& Vector2::operator=(float Val)
{
  X = Val;
  Y = Val;

  return *this;
}

Vector2 Vector2::operator*(float Val) const
{
  return
  {
    X * Val,
    Y * Val
  };
}

Vector2 Vector2::operator*(const Vector2& Vec) const
{
  return
  {
    X * Vec.X,
    Y * Vec.Y
  };
}

Vector2 Vector2::operator+(const Vector2& Vec) const
{
  return
  {
    X + Vec.X,
    Y + Vec.Y
  };
}

Vector2 Vector2::operator-(const Vector2& Vec) const
{
  return
  {
    X - Vec.X,
    Y - Vec.Y
  };
}

Vector2 Vector2::operator/(float Val) const
{
  return
  {
    X / Val,
    Y / Val
  };
}

Vector2 operator-(float Val, const Vector2& Vec)
{
  return
  {
    Val - Vec.X,
    Val - Vec.Y
  };
}

Vector3::Vector3(const struct Quaternion& q) : 
  X(q.X),Y(q.Y),Z(q.Z) { }

void Vector3::Zero()
{
  X = Y = Z = 0;
}

Vector3 Vector3::Norm(float Gain) const
{
  float n = sqrtf(X * X + Y * Y + Z * Z);

  if (n > 1e-12f)
  {
    n = Gain / n;

    return
    {
      X * n,
      Y * n,
      Z * n
    };
  }

  return {0, 0};
}

Vector3 Vector3::Abs() const
{
  return { fabsf(X) , fabsf(Y), fabsf(Z) };
}

float Vector3::Length() const
{
  return sqrtf(X * X + Y * Y + Z * Z);
}

float Vector3::LengthSquared() const
{
  return X * X + Y * Y + Z * Z;
}

Vector3 Vector3::Copy() const
{
  return { X, Y, Z };
}

bool Vector3::IsNAN() const
{
  return (X != X) || (Y != Y) || (Z != Z);
}

bool Vector3::IsFinite() const 
{
  return (X - X == 0) && (Y - Y == 0) && (Z - Z == 0);
}

Vector3 Vector3::Limit(float Max, float Min) const
{
  Vector3 lim;
  
  if (X > Max) lim.X = Max; else if (X < Min) lim.X = Min; else lim.X = X;
  if (Y > Max) lim.Y = Max; else if (Y < Min) lim.Y = Min; else lim.Y = Y;
  if (Z > Max) lim.Z = Max; else if (Z < Min) lim.Z = Min; else lim.Z = Z;
  
  return lim;
}

Vector3 Vector3::Power(float Pow) const
{
  return
  {
    powf(X, Pow),
    powf(Y, Pow),
    powf(Z, Pow)
  };
}

float Vector3::Dot(const Vector3& Vec) const
{
  return X * Vec.X + Y * Vec.Y + Z * Vec.Z;
}

Vector3 Vector3::Cross(const Vector3& Vec) const
{
  return
  {
    Y * Vec.Z - Z * Vec.Y,
    Z * Vec.X - X * Vec.Z,
    X * Vec.Y - Y * Vec.X
  };
}

Vector2 Vector3::GetDirHalfSinCos()
{
  Vector2 xy = Norm();
  
  if(xy.X < -0.999f) { return { 1.0f, 0.0f }; } // Поворот на 180 градусов
  
  float w = sqrtf((1.0f + xy.X) * 0.5f);
  
  return { xy.Y / (2.0f * w), w };
}

Vector3& Vector3::operator+=(const Vector3& Vec)
{
  X += Vec.X;
  Y += Vec.Y;
  Z += Vec.Z;

  return *this;
}

Vector3& Vector3::operator+=(float Val)
{
  X += Val;
  Y += Val;
  Z += Val;

  return *this;
}

Vector3& Vector3::operator-=(const Vector3& Vec)
{
  X -= Vec.X;
  Y -= Vec.Y;
  Z -= Vec.Z;

  return *this;
}

Vector3& Vector3::operator-=(float Val)
{
  X -= Val;
  Y -= Val;
  Z -= Val;

  return *this;
}

Vector3& Vector3::operator*=(float Val)
{
  X *= Val;
  Y *= Val;
  Z *= Val;

  return *this;
}

Vector3& Vector3::operator*=(const Vector3& Vec)
{
  X *= Vec.X;
  Y *= Vec.Y;
  Z *= Vec.Z;

  return *this;
}

Vector3& Vector3::operator/=(float Val)
{
  X /= Val;
  Y /= Val;
  Z /= Val;

  return *this;
}

Vector3& Vector3::operator=(const struct Quaternion& Quat)
{
  X = Quat.X;
  Y = Quat.Y;
  Z = Quat.Z;

  return *this;
}

Vector3& Vector3::operator=(float Val)
{
  X = Val;
  Y = Val;
  Z = Val;

  return *this;
}

Vector3 Vector3::operator*(float Val) const
{
  return
  {
    X * Val,
    Y * Val,
    Z * Val,
  };
}

Vector3 Vector3::operator*(const Vector3& Vec) const
{
  return
  {
    X * Vec.X,
    Y * Vec.Y,
    Z * Vec.Z,
  };
}

Vector3 Vector3::operator+(float Val) const
{
  return
  {
    X + Val,
    Y + Val,
    Z + Val,
  };
}

Vector3 Vector3::operator+(const Vector3& Vec) const
{
  return
  {
    X + Vec.X,
    Y + Vec.Y,
    Z + Vec.Z,
  };
}

Vector3 Vector3::operator-(const Vector3& Vec) const
{
  return
  {
    X - Vec.X,
    Y - Vec.Y,
    Z - Vec.Z,
  };
}

Vector3 Vector3::operator/(float Val) const
{
  return
  {
    X / Val,
    Y / Val,
    Z / Val,
  };
}

Vector3 operator-(float Val, const Vector3& Vec)
{
  return
  {
    Val - Vec.X,
    Val - Vec.Y,
    Val - Vec.Z,
  };
}

#pragma once

#include <functional>


struct Vector2
{
  Vector2(const float v = 0) : X(v), Y(v) {}
  Vector2(const struct Vector3& Vec);
  Vector2(const Vector2& Vec) : X(Vec.X), Y(Vec.Y) {}
  Vector2(float x, float y) : X(x), Y(y) {}

  float X = 0.0f, Y = 0.0f;

  void Zero();

  Vector2 Norm(float Gain = 1.0f) const;
  Vector2 Abs() const;
  float Length() const;
  float LengthSquared() const;
  bool IsNAN() const;
  bool IsFinite() const;
  Vector2 Limit(float Max, float Min) const;
  Vector2 Power(float Pow) const;
  float Dot(const Vector2& Vec) const;
  
  template<typename T> Vector2 Action(T Act) const { return { Act(X), Act(Y) }; };

  Vector2& operator+=(const Vector2& Vec);
  Vector2& operator-=(const Vector2& Vec);
  Vector2& operator*=(float Val);
  Vector2& operator*=(const Vector2& Vec);
  Vector2& operator/=(float Val);

  Vector2& operator=(float Val);

  Vector2 operator*(float Val) const;
  Vector2 operator*(const Vector2& Vec) const;
  Vector2 operator+(const Vector2& Vec) const;
  Vector2 operator-(const Vector2& Vec) const;
  Vector2 operator/(float Val) const;
  
  friend Vector2 operator-(float Val, const Vector2& Vec);
};

struct Vector3
{
  Vector3(const float v = 0) : X(v), Y(v), Z(v) {}
  Vector3(const Vector2& Vec, const float z = 0.0f) : X(Vec.X), Y(Vec.Y), Z(z) {}
  Vector3(const Vector3& Vec) : X(Vec.X), Y(Vec.Y), Z(Vec.Z) {}
  Vector3(float x, float y, float z): X(x), Y(y), Z(z) {}
  Vector3(const struct Quaternion& q);

  float X = 0.0f, Y = 0.0f, Z = 0.0f;

  void Zero();

  Vector3 Norm(float Gain = 1.0f) const;
  Vector3 Abs() const;
  float Length() const;
  float LengthSquared() const;
  Vector3 Copy() const;
  bool IsNAN() const;
  bool IsFinite() const;
  Vector3 Limit(float Max, float Min) const;
  Vector3 Power(float Pow) const;
  float Dot(const Vector3& Vec) const;
  
  template<typename T> Vector3 Action(T Act) const { return { Act(X), Act(Y), Act(Z) }; };

  Vector3 Cross(const Vector3& Vec) const;
  
  Vector2 GetDirHalfSinCos();

  Vector3& operator+=(const Vector3& Vec);
  Vector3& operator+=(float Val);
  Vector3& operator-=(const Vector3& Vec);
  Vector3& operator-=(float Val);
  Vector3& operator*=(float Val);
  Vector3& operator*=(const Vector3& Vec);
  Vector3& operator/=(float Val);

  Vector3& operator=(const struct Quaternion& Quat);
  Vector3& operator=(float Val);

  Vector3 operator*(float Val) const;
  Vector3 operator*(const Vector3& Vec) const;
  Vector3 operator+(float Val) const;
  Vector3 operator+(const Vector3& Vec) const;
  Vector3 operator-(const Vector3& Vec) const;
  Vector3 operator/(float Val) const;

  friend Vector3 operator-(float Val, const Vector3& Vec);
};

#pragma once

#include <functional>


struct Vector2
{
  typedef std::function<float(float)> ActionProc;
  
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
  Vector2 Limit(float Max, float Min) const;
  Vector2 Power(float Pow) const;
  
  Vector2 Action(ActionProc Act) const;

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
  
  friend Vector3 operator-(float Val, const Vector3& Vec);
};

struct Vector3
{
  typedef std::function<float(float)> ActionProc;
  
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
  Vector3 Limit(float Max, float Min) const;
  Vector3 Power(float Pow) const;
  
  Vector3 Action(ActionProc Act) const;

  Vector3 Cross(const Vector3& Vec) const;

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
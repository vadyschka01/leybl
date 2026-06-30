#pragma once

#include "Vector.h"

struct Quaternion
{
  Quaternion() : X(0.0f), Y(0.0f), Z(0.0f), W(1.0f) { }
  Quaternion(float v) : X(v), Y(v), Z(v), W(v) { }
  Quaternion(float x, float y, float z, float w) : X(x), Y(y), Z(z), W(w) { }
  Quaternion(const Vector3& Vec, float w = 0.0f) : X(Vec.X), Y(Vec.Y), Z(Vec.Z), W(w) { }

  float X, Y, Z, W;
  
  void Zero();

  Quaternion Norm(float Gain = 1.0f) const;
  Quaternion Conjugate() const;
  Quaternion Invert() const;
  Quaternion Negate() const;
  bool IsNAN() const;

  Quaternion& operator=(const Quaternion& Q);
  Quaternion& operator+=(const Quaternion& Q);
  Quaternion& operator-=(const Quaternion& Q);
  Quaternion& operator*=(const float Value);
  Quaternion& operator*=(const Quaternion& Q);
  Quaternion operator*(const float Value) const;
  Quaternion operator*(const Quaternion& Q) const;
  Quaternion operator+(const Quaternion& Q) const;
  Quaternion operator-(const Quaternion& Q) const;

  Vector3 Rotate(const Vector3& vec) const;
  Vector3 RotateAroundZ(const Vector3& vec, bool CCW = false) const;
  
  static Quaternion CreateYawPitchRoll(const Vector3& PitchRollYawHalfSin, const Vector3* PitchRollYawHalfCos, const float* YawHalfCos);
  static Quaternion CreateYawPitchRoll(const Vector3& PitchRollYawRad);
  static Quaternion CreatePitchRollYaw(const Vector3& PitchRollYawHalfSin, const Vector3* PitchRollYawHalfCos, const float* YawHalfCos);
  static Quaternion CreatePitchRollYaw(const Vector3& PitchRollYawRad);
  static Quaternion CreateYaw(const float YawRad);
  
  Quaternion GetError(const Quaternion& Target, bool FastWay) const;
};

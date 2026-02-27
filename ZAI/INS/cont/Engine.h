#pragma once

#include "../geom/Vector.h"
#include "../util/Record.h"
#include "../geom/Quaternion.h"

class Engine
{
public:
  Engine(const Vector3& xyz) :OriPRT(xyz) {};

public:
  unsigned char QuadSchemeX[4]
  {// CCW   FRONT  CW
        2,         0,
        1,         3
  };// CW   REAR   CCW

private:
  const Vector3& OriPRT; // Наклоны SinX SinY CosZ из IRS

  float ThrustPitch, ThrustRoll, ThrustYaw, ThrustThrot;

  float TiltСomp = 0.7f;

public:
  void SetTiltThrot(float Tilt);

  void SetPowerPRY(const Vector3& PowerPRY, float Throt);

  void GetQuadX(float Thrust[4]);
};
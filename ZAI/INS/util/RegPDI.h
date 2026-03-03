#pragma once

struct RegPDI
{
  float Period;

  float Lim = 0; // Ограничение

  float Pro = 0;

  float Der = 0;

  struct // Integral
  {
    float Lim = 0;

    float Pro = 0;
    float Der = 0;
  } Int;

  bool LockI = true;

  float I = 0;

  float Update(float Value, float Current, float Derivative);
};


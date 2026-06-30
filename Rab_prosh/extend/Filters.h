#pragma once
#include "INS/geom/Vector.h"
#include <math.h>

struct ExpCoef
{
  float init; // начальное значение коэф
  float max; // максимальное значение коэф
  float d; // Коэф значимости новых измерений
};

struct ExpAvgSmoothFilterBaro
{
  float total;
  bool initialized = false;
  ExpCoef coef = { 0.01f, 0.15f, 2.4f };
  static const int window = 25;
  float buffer[window];
  bool active = false;
  int index = 0;
  float filteredStore; // Контекст
  float update(float value);
  float update2(float value);
  void reinit(float value);
};

class BiquadFilter2
{
public:
  BiquadFilter2();
  
  float a0,a1,a2;
  float b1,b2;
  
  float process(float value);
  bool active;
  void reset();
  
private:
  float x1,x2;
  float y1,y2;
};

class ButterworthCascadeFilter
{
public:
  ButterworthCascadeFilter(float sampleRate);
  void reset();
  float process(float value);
  
private:
  static const int order = 6;
  
  const float cutOffFreq = 4.0f; // Hz srez
  
  BiquadFilter2 biquads[order/2];
};

extern ExpAvgSmoothFilterBaro BaroFilt;


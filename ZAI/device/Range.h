#pragma once

#include "../geom/Vector.h"
#include "../util/Record.h"
#include "../util/RegPDI.h"
#include "../geom/Quaternion.h"

class Range
{
public:
  Range(const Record<Vector3>& RecordSpd, const Record<Quaternion>& RecordQua);

public:
  static constexpr float Freq = 25.0f;
  static constexpr float Period = 1.0f / Freq;
  
  float MaxHeight = 7.5f; // Максимальная высота измерения
  float MinHeight = 0.02f; // Максимальная высота измерения

  float MaxAngle = 0.7f; // Угол наклона отсутствия доверия 45
  
  float MaxSpeedXY = 10.0f; // Максимальная скорость движения для коррекции скорости высоты
  
  bool AbsolutePosMode = true; // Режим принудительного контроля высоты
  
  Average<Vector3> AveSpd;
  
public:
  bool Active;
  
  float Len;

  float Past = 0.0f;

  const Record<Vector3>& RecSpd;
  const Record<Quaternion>& RecQua;

  
  struct { bool Ready; float Spd, Prv, Dyn; } Shift;

public:
  struct { float Restore, Accuracy, Power, Test;  } RegShift; // Регулятор смещения позиции и скорости
  struct { float Alpha, Power, Dynamic, Distance; } RegSpd; // Регулятор смещения скорости
  
public:
  void UpdateAverage();
  
  float SetRange(float* Range);
  
  bool GetShift(float& SpeedZ);
};

#pragma once

#include "../geom/Vector.h"
#include "../util/Record.h"
#include "../util/RegPDI.h"
#include "../geom/Quaternion.h"

class Barometer
{
public:
  Barometer(const Record<Vector3>& RecordPos, const Record<Vector3>& RecordSpd, const Record<Quaternion>& RecordQua);
  float Gravity = 0.01f; // Коэффициент восстановления угла
  
  float Distance = 3.0f; // Дистанция измерений alpha
  float Power = 2;       // Степень alpha

public:
  static constexpr float Freq = 50.0f;
  static constexpr float Period = 1.0f / Freq;

  bool Active;

  float Temp, Bar, Zero;
  float LastPos;

  float Past = 0.5f;

  Average<float> AveSpd;

  const Record<Vector3>& RecPos;
  const Record<Vector3>& RecSpd;

  struct { bool Ready; float Pos, Spd, Prv, Dyn; } Shift;
  struct { Quaternion Qua; } Quat;

  void UpdateQuaternion();
  
public:
  struct { float Restore, Accuracy, Power, Test; } RegShift; // Регулятор смещения позиции и скорости
  struct { float Alpha, Power, Dynamic, Distance; } RegSpd; // Регулятор смещения скорости
  struct { float Alpha, Power, Dynamic, Distance; } RegPos; // Регулятор смещения позиции
  
  struct
  {
    float AlphaMax = 0.02f;
    float AlphaMin = 0.005f;
    
    float AccuracyMin = 0.01f;
    
    float AccuracyMax = 0.1f;
  } Adaptive;

public:
  void UpdateAverage();

  void SetPressure(const float* Pressure, const float* Temperature);
  void UpdateZeroHeight(float ZeroHeight, float Alpha);

public:
  void GetTempBarZero(float* Temperature, float* Pressure, float* ZeroPressure);

  bool GetShift(float& PositionZ, float& SpeedZ);
};

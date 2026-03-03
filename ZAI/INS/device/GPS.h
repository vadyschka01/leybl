#pragma once

#include "../geom/Vector.h"
#include "../util/Record.h"
#include "../util/RegPDI.h"

class GPS
{
public:
  GPS(const Record<Vector3>& RecordPos, const Record<Vector3>& RecordSpd);
  
  static constexpr float Freq = 100.0f;
  static constexpr float Period = 1.0f / Freq;

  float Wait = 1.0f; // Допустимое ожидание координат
  
  float MeasureAccuracy = 1.5f; // Точность измерений [м]
  float MeasureAlphaPos = 0.5f; // Максимальное доверие к позиции GPS
  float MeasureAlphaSpd = 0.5f; // Максимальное доверие к скорости GPS

private:
  bool Active;

  float Timer = 0;
  Vector3 LastPos, LastSpd;

  
  float Past = 0.0f;

  Average<Vector3> AveSpd;

  const Record<Vector3>& RecPos;
  const Record<Vector3>& RecSpd;

public:
  struct { bool Ready; Vector3 Pos, Spd, Prv, Dyn; } Shift;

public:
  struct { float Restore, Accuracy, Power, Test; } RegShift; // Регулятор смещения позиции и скорости
  struct { float Alpha, Power, Dynamic, Distance; } RegSpd; // Регулятор смещения скорости
  struct { float Alpha, Power, Dynamic, Distance; } RegPos; // Регулятор смещения позиции
  float AlphaHeight; // Регулятор смещения высоты
  
public:
  void UpdateAverage(float Time);

  void SetPosition(const Vector3* Position, float Time);

public:
  bool GetTime(float& Time);
  bool GetPositionSpeed(Vector3* Position, Vector3* Speed);

  bool GetShift(Vector3& Position, Vector3& Speed);
};

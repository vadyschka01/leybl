#pragma once

#include "../geom/Vector.h"
#include "../util/Record.h"
#include "../util/RegPDI.h"

class OpticalFlow
{
public:
  OpticalFlow(const Record<Vector3>& RecordGyr, const Record<Vector3>& RecordSpd, const Record<Vector3>& RecordPos, const Record<Quaternion>& RecordQua);

  float Lens = 0.0018f; // Компенсация линзы
  
  float MinAngle = 30.0f; // Угол наклона потери доверия
  float MaxAngle = 45.0f; // Угол наклона отсутствия доверия

  float MinSpeed = 0.1f;  // Уголовая скорость потери доверия
  float MaxSpeed = 0.5f; // Уголовая скорость отсутствия доверия
  
  float MinQuality = 0.08f; // Минимальное качество оптического потока
  float MaxQuality = 0.25f; // Достаточное качество оптического потока
  
  float MinHeight = 4.00f; // Высота потери доверия
  float MaxHeight = 7.00f; // Высота отсутствия доверия
  
  static constexpr float Freq = 25.0f;
  
  Vector2 TEST_SHIFT;

private:
  static constexpr float Period = 1.0f / Freq;

  bool Active;

  float SubLen;

  float Past = 0.02f;

  struct StructRecOF
  {
    static constexpr float Past = 0.2f; // Максимальная запись прошлых данных
    static constexpr unsigned long Count = Freq * Past + 1;
    Vector2 Buffer[Count];
    Record<Vector2> Rec;
  } RecOF;

  Vector2 AveGyrMove;
  Average<Vector2> AveSpd;

  const Record<Vector3>& RecGyr;
  const Record<Vector3>& RecSpd;
  const Record<Vector3>& RecPos;
  const Record<Quaternion>& RecQua;
  Record<Vector2>& RecordOF;

public:
  struct { bool Ready; Vector2 Pos, Spd, Prv, Dyn; } Shift;
  
public:
  struct { float Restore, Accuracy, Power, Test;  } RegShift; // Регулятор смещения позиции и скорости
  struct { float Alpha, Power, Dynamic, Distance; } RegSpd; // Регулятор смещения скорости

public:
  void UpdateAverage(float InertialPeriod);

  void SetMove(const Vector2* Move, float Range, float Quality);

public:
  bool GetShift(Vector2& Position, Vector2& Speed);
};

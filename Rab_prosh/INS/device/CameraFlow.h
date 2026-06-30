#pragma once

#include "../geom/Vector.h"
#include "../util/Record.h"
#include "../util/RegPDI.h"

class CameraFlow
{
public:
  CameraFlow(const Record<Vector3>& RecordSpd, const Record<Vector3>& RecordPos, const Record<Quaternion>& RecordQua);

  float Lens = 1.15E-3f; // Компенсация линзы

private:

  bool Active;

  float SubLen;

  Average<Vector2> AveSpd;

  const Record<Vector3>& RecSpd;
  const Record<Vector3>& RecPos;
  const Record<Quaternion>& RecQua;
  
  unsigned long LastTime;

public:
  struct { bool Ready; Vector2 Pos, Spd, Prv, Dyn; } Shift;
  
public:
  struct { float Restore, Accuracy, Power, Test;  } RegShift; // Регулятор смещения позиции и скорости
  struct { float Alpha, Power, Dynamic, Distance; } RegSpd; // Регулятор смещения скорости

public:
  void UpdateAverage();
  
  void SetMove(const Vector2* Move, float Range, unsigned long Time);

public:
  bool GetShift(Vector2& Position, Vector2& Speed);
};

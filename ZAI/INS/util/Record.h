#pragma once

#include "../geom/Vector.h"
#include "../geom/Quaternion.h"

template <typename T>
class Record
{
public:
  Record() {};
  Record(T* RecordBuffer, float Frequency, float Period); // BufferSize = (Frequency * Period) + 1;

private:
  float Freq;
  unsigned long Count;
  T* Buffer;
  T Shift;
  unsigned long Index = 0;

public:
  void Init(T* RecordBuffer, float Frequency, float Period); // BufferSize = (Frequency * Period) + 1;

  T Past(float Time) const;

  void Add(const T& Value);

  void Mix(const T& Shift);
};

template <typename T>
class Average
{
private:
  T Value;
  float Count = 0;

public:
  void Set(const T& Val) { Value += Val; Count++; }
  T Get() { if (Count == 0.0f) return T(); T ave = Value / Count; Count = 0; Value = 0.0f; return ave; }
};

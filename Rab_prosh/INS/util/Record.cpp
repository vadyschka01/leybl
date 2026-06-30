#include <math.h>

#include "Record.h"

template class Record<float>;
template class Record<Vector2>;
template class Record<Vector3>;
template class Record<Quaternion>;

template <class T>
Record<T>::Record(T* RecordBuffer, float Frequency, float Period)
{
  Init(RecordBuffer, Frequency, Period);
}

template <class T>
void Record<T>::Init(T* RecordBuffer, float Frequency, float Period)
{
  Freq = Frequency;
  Buffer = RecordBuffer;
  Count = (float)(Frequency * Period + 1.0f);
}

template <class T>
T Record<T>::Past(float Time) const
{
  unsigned long move = (float)(Time * Freq);
  if (move >= Count) move = Count - 1;
  
  if (Index >= move) move = Index - move;
  else move = Index + (Count - move);

  return Buffer[move];
}

template <class T>
void Record<T>::Add(const T& Value)
{
  Index++;
  if (Index >= Count) Index = 0;
  Buffer[Index] = Value;
}

template <class T>
void Record<T>::Mix(const T& Shift)
{
  for (unsigned int a = 0; a < Count; a++) Buffer[a] += Shift;
}

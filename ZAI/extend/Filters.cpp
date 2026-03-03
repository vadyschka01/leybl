#include "Filters.h"

ExpAvgSmoothFilterBaro BaroFilt;

float ExpAvgSmoothFilterBaro::update2(float value)
{
  // --- Инициализация при первом вызове ---
  if (!initialized)
  {
    // Заполняем весь буфер первым значением, чтобы избежать "разогрева"
    filteredStore = value;
    for (int i = 0; i < window; i++)
    {
      buffer[i] = filteredStore;
    }
    total = filteredStore * window;
    initialized = true;
  }

  // --- Этап 1: Адаптивный экспоненциальный фильтр (логика осталась той же) ---
  if (fabsf(value - filteredStore) < coef.d) {
    filteredStore += (value - filteredStore) * coef.init;
  }
  else {
    filteredStore += (value - filteredStore) * coef.max;
  }

  // --- Этап 2: Эффективное скользящее среднее на кольцевом буфере ---

  // 1. Вычитаем самое старое значение (которое собираемся заменить) из общей суммы
  total -= buffer[index];

  // 2. Заменяем самое старое значение новым (отфильтрованным на этапе 1)
  buffer[index] = filteredStore;

  // 3. Добавляем новое значение к общей сумме
  total += buffer[index];

  // 4. Сдвигаем индекс для следующей итерации (циклически)
  index = (index + 1) % window;

  // 5. Возвращаем новое среднее
  return total / window;
}

BiquadFilter::BiquadFilter()
{
  reset();
}

float BiquadFilter::process(float value)
{
  if(active)
  {
    float out = a0 * value + a1*x1 + a2*x2 - b1*y1 - b2*y2;
  
    x2 = x1;
    x1 = value;
    y2 = y1;
    y1 = out;
  
    return out;
  }
  else
  {
    active = true;
    x1 = x2 = y1 = y2 = value;
    return value;
  }
}

void BiquadFilter::reset()
{
  x1 = x2 = y1 = y2 = 0.0f;
  active = false;
}

ButterworthCascadeFilter::ButterworthCascadeFilter(float sampleRate)
{
  float wc = tanf(3.14159265359f * cutOffFreq / sampleRate);
  float k = wc;
  float k2 = k*k;
  
  for (int i = 0; i < order/2; i++)
  {
    float phi = 3.14159265359f * (2.0f * (i + 1.0f) - 1.0f) / (2.0f * order);
    float alpha = sinf(phi);
    float denominator = 1.0f + 2.0f * alpha * k + k2;
    
    biquads[i].a0 = k2/denominator;
    biquads[i].a1 = 2.0f * biquads[i].a0;
    biquads[i].a2 = biquads[i].a0;
    biquads[i].b1 = 2.0f * (k2 - 1.0f) / denominator;
    biquads[i].b2 = (1.0f - 2.0f * alpha * k + k2) / denominator;
  }
}

float ButterworthCascadeFilter::process(float value)
{
  float out = value;
  for (int i = 0; i < order/2; i++)
  {
    if (i == 0) out = biquads[i].process(out);
    else
    {
      if (biquads[i-1].active) out = biquads[i].process(out);
    }
  }
  return out;
}

void ButterworthCascadeFilter::reset()
{
  for (int i = 0; i < order/2; i++)
  {
    biquads[i].reset();
  }
}
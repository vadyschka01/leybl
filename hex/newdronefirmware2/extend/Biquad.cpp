#include "Biquad.h"

static constexpr float PI = 3.14159265359f;

// Функция для инициализации фильтра
// Рассчитывает коэффициенты на основе нужных параметров
void lpf2p_init(BiquadFilter *filter, float sample_freq, float cutoff_freq)
{
    if (cutoff_freq <= 0.0f || sample_freq <= 0.0f)
    {
        // Некорректные параметры, фильтр не будет работать
        return;
    }

    float fr = sample_freq / cutoff_freq;
    float ohm = tanf(PI / fr);
    float c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;

    filter->b0 = ohm * ohm / c;
    filter->b1 = 2.0f * filter->b0;
    filter->b2 = filter->b0;

    filter->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    filter->a2 = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;

    // Сбрасываем историю
    filter->x1 = filter->x2 = 0.0f;
    filter->y1 = filter->y2 = 0.0f;
}

// Функция для инициализации фильтра
// Рассчитывает коэффициенты на основе нужных параметров
void notch_init(BiquadFilter *filter, float sample_freq, float center_freq, float bandwidth)
{
    if (center_freq <= 0.0f || sample_freq <= 0.0f || bandwidth <= 0.0f)
    {
        return; // Некорректные параметры
    }

    // Промежуточные расчеты по стандартным формулам для biquad-фильтра
    float w0 = 2.0f * PI * center_freq / sample_freq;
    float alpha = sinf(w0) * sinhf(logf(2.0f) / 2.0f * bandwidth * w0 / sinf(w0));

    // Рассчитываем коэффициенты b (для входа x) и a (для выхода y)
    float b0_temp = 1.0f;
    float b1_temp = -2.0f * cosf(w0);
    float b2_temp = 1.0f;
    
    float a0_temp = 1.0f + alpha;
    float a1_temp = -2.0f * cosf(w0);
    float a2_temp = 1.0f - alpha;

    // Нормализуем коэффициенты (делим на a0), чтобы использовать в уравнении
    filter->b0 = b0_temp / a0_temp;
    filter->b1 = b1_temp / a0_temp;
    filter->b2 = b2_temp / a0_temp;
    
    filter->a1 = a1_temp / a0_temp;
    filter->a2 = a2_temp / a0_temp;

    // Сбрасываем историю
    filter->x1 = filter->x2 = 0.0f;
    filter->y1 = filter->y2 = 0.0f;
}

// Функция применения фильтра к новому значению
float biquad_apply(BiquadFilter *filter, float input)
{
    // Применяем разностное уравнение: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float output = filter->b0 * input +
                   filter->b1 * filter->x1 +
                   filter->b2 * filter->x2 -
                   filter->a1 * filter->y1 -
                   filter->a2 * filter->y2;

    // Обновляем историю для следующего шага
    filter->x2 = filter->x1;
    filter->x1 = input;

    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}


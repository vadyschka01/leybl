#pragma once

#include "INS/geom/Vector.h"
#include <math.h>

// Структура для хранения состояния и коэффициентов фильтра
typedef struct
{
    // Коэффициенты фильтра
    float a1, a2;
    float b0, b1, b2;

    // "Элементы задержки" - хранят предыдущие значения
    float x1, x2; // предыдущие входы
    float y1, y2; // предыдущие выходы
} BiquadFilter;

void lpf2p_init(BiquadFilter *filter, float sample_freq, float cutoff_freq);

void notch_init(BiquadFilter *filter, float sample_freq, float center_freq, float bandwidth);

float biquad_apply(BiquadFilter *filter, float input);
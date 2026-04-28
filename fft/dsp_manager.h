#ifndef DSP_MANAGER_H
#define DSP_MANAGER_H

#include "arm_math.h"

// Размер окна Фурье (степень двойки)
#define FFT_SIZE 512

// Прототипы
void DSP_Init(void);
void DSP_AddSample(float32_t sample); // Добавить одну точку в "копилку"
void DSP_Process(void);              // Запустить расчет (когда накопили 1024)

// Флаг готовности данных (чтобы main знал, когда пора вызывать Process)
extern uint8_t dsp_buffer_ready;

#endif
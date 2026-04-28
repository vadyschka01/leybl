#ifndef IMU_H
#define IMU_H

#include "stm32g4xx.h"

#define IMU_ADDR (0x68 << 1)

typedef struct {
    float b0, b1, b2, a1, a2;
    float d1, d2;
} biquad_t;

// Делаем фильтры видимыми для dsp_manager
extern biquad_t notch1, notch2, notch3;

// Прототипы
void I2C1_Init(void);
void IMU_Init(void);
void IMU_Calibrate(void);
void IMU_ReadRawData(void);
void biquad_init_notch(biquad_t *f, float center_freq, float Q, float fs);

// Данные
extern volatile int16_t raw_gx; // Нам для анализа нужен только GX
extern float filt_gx;

#endif
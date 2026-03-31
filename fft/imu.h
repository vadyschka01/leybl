#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#define IMU_ADDR (0x68 << 1)

// Структура фильтра (Биквад)
typedef struct {
    float b0, b1, b2, a1, a2;
    float d1, d2;
} biquad_t;

// Глобальные сырые данные (для логгера)
extern volatile int16_t raw_gx, raw_gy, raw_gz;
extern volatile int16_t raw_ax, raw_ay, raw_az;

// Отфильтрованные данные (в deg/sec и g)
extern float filt_gx, filt_gy, filt_gz;

// Функции
void I2C1_Init(void);
void IMU_Init(void);
void IMU_ReadRaw(void);
void IMU_Calibrate(void);

// Инициализация фильтров
void biquad_init_lpf(biquad_t *f, float cutoff, float fs);
void biquad_init_notch(biquad_t *f, float center_freq, float Q, float fs);
float biquad_apply(biquad_t *f, float x);

#endif
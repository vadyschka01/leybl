#ifndef IMU_H
#define IMU_H

#include "stm32g4xx.h"

#define IMU_ADDR (0x68 << 1)

typedef struct {
    float b0, b1, b2, a1, a2;
    float d1, d2;
} biquad_t;

// Старые фильтры notch1,2,3 удалены (использовалась только FMAC с notch_fmac_coeffs[3])
// extern biquad_t notch1, notch2, notch3;

// Прототипы
void I2C1_Init(void);
void IMU_Init(void);
void IMU_Calibrate(void);
void IMU_ReadRawData(void);
// biquad_init_notch удалена (заменена на Update_FMAC_Coeffs в dsp_manager)

// Данные
extern volatile int16_t raw_gx; // Нам для анализа нужен только GX
extern float filt_gx;

// Добавить в imu.h
typedef struct {
    int16_t b0, b1, b2; // Коэффициенты числителя
    int16_t a1, a2;     // Коэффициенты знаменателя (инвертированные для FMAC)
} fmac_weights_t;

void FMAC_Init(void);
float FMAC_Process_Sample(float input);
void Update_FMAC_Coeffs(int stage, float b0, float b1, float b2, float a1, float a2);

typedef struct {
    int16_t b0, b1, b2;
    int16_t a1, a2;
} fmac_coeffs_t;

typedef struct {
    int16_t x1, x2; // История входов (x[n-1], x[n-2])
    int16_t y1, y2; // История выходов (y[n-1], y[n-2])
} fmac_state_t;

// Внешние структуры для 3-х каскадов
extern fmac_coeffs_t notch_fmac_coeffs[3];
extern fmac_state_t  notch_fmac_state[3];

#endif
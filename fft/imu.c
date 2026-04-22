#include "stm32g4xx.h"
#include "imu.h"
#include <math.h>

// Глобальные переменные сырых данных
volatile int16_t raw_ax, raw_ay, raw_az;
volatile int16_t raw_gx, raw_gy, raw_gz;

// Глобальные переменные фильтрованных данных
float filt_gx, filt_gy, filt_gz;

// Смещения (Bias)
float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

// Структуры фильтров (Notch для всех трех осей)
static biquad_t lpf_gx, notch1_gx, notch2_gx, notch3_gx;
static biquad_t lpf_gy, notch1_gy, notch2_gy, notch3_gy;
static biquad_t lpf_gz, notch1_gz, notch2_gz, notch3_gz;

// --- МАТЕМАТИКА БИКВАДОВ (Не меняем) ---
float biquad_apply(biquad_t *f, float x) {
    float out = f->b0 * x + f->d1;
    f->d1 = f->b1 * x - f->a1 * out + f->d2;
    f->d2 = f->b2 * x - f->a2 * out;
    return out;
}

void biquad_init_notch(biquad_t *f, float center_freq, float Q, float fs) {
    float w0 = 2.0f * 3.14159265f * center_freq / fs;
    float alpha = sinf(w0) / (2.0f * Q);
    float cosw0 = cosf(w0);
    float a0 = 1.0f + alpha;
    f->b0 = 1.0f / a0;
    f->b1 = -2.0f * cosw0 / a0;
    f->b2 = 1.0f / a0;
    f->a1 = -2.0f * cosw0 / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->d1 = 0; f->d2 = 0;
}

void biquad_init_lpf(biquad_t *f, float cutoff, float fs) {
    float w0 = 2.0f * 3.14159f * cutoff / fs;
    float cosw0 = cosf(w0);
    float alpha = sinf(w0) / (2.0f * 0.707f);
    float a0 = 1.0f + alpha;
    f->b0 = ((1.0f - cosw0) / 2.0f) / a0;
    f->b1 = (1.0f - cosw0) / a0;
    f->b2 = ((1.0f - cosw0) / 2.0f) / a0;
    f->a1 = (-2.0f * cosw0) / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->d1 = f->d2 = 0;
}

// --- ВАШ КОД I2C (Без изменений) ---
void I2C1_Init(void) { /* ... ваш код ... */ }
void I2C_ReadMulti(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint8_t len) { /* ... ваш код ... */ }
static void IMU_WriteReg(uint8_t reg, uint8_t val) { /* ... ваш код ... */ }
void IMU_SetBank(uint8_t bank) { IMU_WriteReg(0x7F, (bank & 0x03) << 4); }

// --- ИНИЦИАЛИЗАЦИЯ ДАТЧИКА И ФИЛЬТРОВ ---
void IMU_InitFilters(void) {
    // Датчик
    IMU_SetBank(0);
    IMU_WriteReg(0x06, 0x80); // Reset
    for(volatile int i=0; i<200000; i++);
    IMU_WriteReg(0x06, 0x01); // Wake
    for(volatile int i=0; i<200000; i++);
    IMU_WriteReg(0x07, 0x00); // Gyro+Accel ON
    IMU_SetBank(2);
    
    // DLPF включен для всех (по даташиту ICM-20948)
    IMU_WriteReg(0x01, (3 << 5) | (1 << 4) | (3 << 2) | 1); // FCHOICE=1
    IMU_WriteReg(0x14, (3 << 5) | (1 << 4) | (0 << 2) | 1); // Accel
    
    IMU_SetBank(0);
    
    // Фильтры (Placeholder частоты — настроим позже, Q=1.0 широкий фильтр)
    float fs = 1000.0f;
    
    // Ось X
    biquad_init_notch(&notch1_gx, 477.0f, 0.5f, fs);
    biquad_init_notch(&notch2_gx, 242.0f, 0.5f, fs);
    biquad_init_notch(&notch3_gx, 125.0f, 0.5f, fs);
    biquad_init_lpf(&lpf_gx, 100.0f, fs);
    
    // Ось Y
    biquad_init_notch(&notch1_gy, 477.0f, 0.5f, fs);
    biquad_init_notch(&notch2_gy, 242.0f, 0.5f, fs);
    biquad_init_notch(&notch3_gy, 125.0f, 0.5f, fs);
    biquad_init_lpf(&lpf_gy, 100.0f, fs);
    
    // Ось Z
    biquad_init_notch(&notch1_gz, 477.0f, 0.5f, fs);
    biquad_init_notch(&notch2_gz, 242.0f, 0.5f, fs);
    biquad_init_notch(&notch3_gz, 125.0f, 0.5f, fs);
    biquad_init_lpf(&lpf_gz, 100.0f, fs);
}

// --- КАЛИБРОВКА (Исправлено: берем правильные индексы буфера) ---
void IMU_Calibrate(void) {
    const int samples = 512;
    int32_t gx_s = 0, gy_s = 0, gz_s = 0;
    uint8_t buf[14];

    for (int i = 0; i < samples; i++) {
        I2C_ReadMulti(IMU_ADDR, 0x2D, buf, 14);
        
        // Правильные индексы гироскопов: 8,10,12 (температура 6,7 пропущены)
        gx_s += (int16_t)(buf[8] << 8 | buf[9]);
        gy_s += (int16_t)(buf[10] << 8 | buf[11]);
        gz_s += (int16_t)(buf[12] << 8 | buf[13]);
        
        for (volatile int d = 0; d < 2000; d++);
    }
    gyro_bias_x = (float)gx_s / samples;
    gyro_bias_y = (float)gy_s / samples;
    gyro_bias_z = (float)gz_s / samples;
}

// --- ЧТЕНИЕ И ФИЛЬТРАЦИЯ (Для всех трех осей) ---
void IMU_ReadRawData(void) {
    uint8_t buf[14];
    I2C_ReadMulti(IMU_ADDR, 0x2D, buf, 14);

    raw_ax = (int16_t)(buf[0] << 8 | buf[1]);
    raw_ay = (int16_t)(buf[2] << 8 | buf[3]);
    raw_az = (int16_t)(buf[4] << 8 | buf[5]);
    
    // Гироскопы: начинаем с 8-го байта
    raw_gx = (int16_t)(buf[8] << 8 | buf[9]);
    raw_gy = (int16_t)(buf[10] << 8 | buf[11]);
    raw_gz = (int16_t)(buf[12] << 8 | buf[13]);

    // --- ОБРАБОТКА ГИРОСКОПА X ---
    float x = (float)raw_gx - gyro_bias_x;
    x = biquad_apply(&notch1_gx, x);
    x = biquad_apply(&notch2_gx, x);
    x = biquad_apply(&notch3_gx, x);
    x = biquad_apply(&lpf_gx, x);
    filt_gx = x;

    // --- ОБРАБОТКА ГИРОСКОПА Y ---
    float y = (float)raw_gy - gyro_bias_y;
    y = biquad_apply(&notch1_gy, y);
    y = biquad_apply(&notch2_gy, y);
    y = biquad_apply(&notch3_gy, y);
    y = biquad_apply(&lpf_gy, y);
    filt_gy = y;

    // --- ОБРАБОТКА ГИРОСКОПА Z ---
    float z = (float)raw_gz - gyro_bias_z;
    z = biquad_apply(&notch1_gz, z);
    z = biquad_apply(&notch2_gz, z);
    z = biquad_apply(&notch3_gz, z);
    z = biquad_apply(&lpf_gz, z);
    filt_gz = z;
}
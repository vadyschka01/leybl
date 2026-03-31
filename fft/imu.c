#include "stm32g4xx.h"
#include "imu.h"
#include <math.h>

// ---------------- RAW DATA (Для передачи на ПК) ----------------
volatile int16_t raw_ax, raw_ay, raw_az;
volatile int16_t raw_gx, raw_gy, raw_gz;

// ---------------- FILTERED DATA ----------------
float filt_gx, filt_gy, filt_gz;

// ---------------- OFFSETS / BIAS ----------------
float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

// ---------------- ФИЛЬТРЫ ----------------
// Каскад для гироскопа X (LPF + 3 Notch)
static biquad_t lpf_gx;
static biquad_t notch1_gx, notch2_gx, notch3_gx;

// --- Твои рабочие функции I2C ---
void I2C1_Init(void) {
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
    GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOB->MODER |=  (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);
    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
    GPIOB->PUPDR  |= (GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0);
    GPIOB->AFR[1] |= (4 << 0) | (4 << 4);
    I2C1->TIMINGR = 0x00303D5B;
    I2C1->CR1 |= I2C_CR1_PE;
}

static void IMU_WriteReg(uint8_t reg, uint8_t val) {
    I2C1->CR2 = (IMU_ADDR & I2C_CR2_SADD) | (2 << 16) | I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = reg;
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = val;
    while (!(I2C1->ISR & I2C_ISR_TC));
    I2C1->CR2 |= I2C_CR2_STOP;
}

void IMU_SetBank(uint8_t bank) {
    IMU_WriteReg(0x7F, (bank & 0x03) << 4);
}

// --- Математика Биквадов (LPF и Notch) ---
float biquad_apply(biquad_t *f, float x) {
    float result = f->b0 * x + f->d1;
    f->d1 = f->b1 * x - f->a1 * result + f->d2;
    f->d2 = f->b2 * x - f->a2 * result;
    return result;
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

void biquad_init_notch(biquad_t *f, float center_freq, float Q, float fs) {
    float w0 = 2.0f * 3.14159f * center_freq / fs;
    float cosw0 = cosf(w0);
    float alpha = sinf(w0) / (2.0f * Q);
    float a0 = 1.0f + alpha;
    f->b0 = 1.0f / a0;
    f->b1 = (-2.0f * cosw0) / a0;
    f->b2 = 1.0f / a0;
    f->a1 = (-2.0f * cosw0) / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->d1 = f->d2 = 0;
}

// --- Твоя калибровка ---
void IMU_Calibrate(void) {
    const int samples = 500;
    int32_t s_gx = 0, s_gy = 0, s_gz = 0;
    uint8_t buf[14];

    for (int i = 0; i < samples; i++) {
        IMU_SetBank(0);
        I2C1->CR2 = (IMU_ADDR & I2C_CR2_SADD) | (1 << 16) | I2C_CR2_START;
        while (!(I2C1->ISR & I2C_ISR_TXIS));
        I2C1->TXDR = 0x2D;
        while (!(I2C1->ISR & I2C_ISR_TC));
        I2C1->CR2 = (IMU_ADDR & I2C_CR2_SADD) | (14 << 16) | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;
        for (int j = 0; j < 14; j++) {
            while (!(I2C1->ISR & I2C_ISR_RXNE));
            buf[j] = I2C1->RXDR;
        }
        s_gx += (int16_t)(buf[6] << 8 | buf[7]);
        s_gy += (int16_t)(buf[8] << 8 | buf[9]);
        s_gz += (int16_t)(buf[10] << 8 | buf[11]);
        for (volatile int d = 0; d < 5000; d++);
    }
    gyro_bias_x = (float)s_gx / samples;
    gyro_bias_y = (float)s_gy / samples;
    gyro_bias_z = (float)s_gz / samples;
}

// --- Инициализация ---
void IMU_Init(void) {
    IMU_SetBank(0);
    IMU_WriteReg(0x06, 0x80);
    for(volatile int i=0; i<100000; i++);
    IMU_WriteReg(0x06, 0x01);
    IMU_WriteReg(0x07, 0x00);
    IMU_SetBank(2);
    IMU_WriteReg(0x01, (3 << 3) | (3 << 1) | 0); // 73Hz LPF, 2000dps
    IMU_WriteReg(0x14, (3 << 3) | (0 << 1) | 0); // 73Hz LPF, 2g
    IMU_SetBank(0);

    // Инициализация фильтров (LPF обязателен, Notch пока пустые)
    float fs = 1000.0f;
    biquad_init_lpf(&lpf_gx, 100.0f, fs);
}

// --- Чтение и фильтрация ---
void IMU_ReadRaw(void) {
    uint8_t buf[14];
    IMU_SetBank(0);
    I2C1->CR2 = (IMU_ADDR & I2C_CR2_SADD) | (1 << 16) | I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = 0x2D;
    while (!(I2C1->ISR & I2C_ISR_TC));
    I2C1->CR2 = (IMU_ADDR & I2C_CR2_SADD) | (14 << 16) | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;
    for (int i = 0; i < 14; i++) {
        while (!(I2C1->ISR & I2C_ISR_RXNE));
        buf[i] = I2C1->RXDR;
    }

    raw_ax = (int16_t)(buf[0] << 8 | buf[1]);
    raw_ay = (int16_t)(buf[2] << 8 | buf[3]);
    raw_az = (int16_t)(buf[4] << 8 | buf[5]);
    raw_gx = (int16_t)(buf[6] << 8 | buf[7]);
    raw_gy = (int16_t)(buf[8] << 8 | buf[9]);
    raw_gz = (int16_t)(buf[10] << 8 | buf[11]);

    // Обработка Gyro X
    float gx = (float)raw_gx - gyro_bias_x;
    
    // Сюда добавим Notch, когда узнаем частоты
    // gx = biquad_apply(&notch1_gx, gx);
    
    filt_gx = biquad_apply(&lpf_gx, gx);
}
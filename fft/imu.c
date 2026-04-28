#include "imu.h"
#include <math.h>

volatile int16_t raw_ax, raw_ay, raw_az;
volatile int16_t raw_gx, raw_gy, raw_gz;
float filt_gx;
float gyro_bias_x = 0;

// Сами фильтры
biquad_t notch1, notch2, notch3;

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

void I2C1_Init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
    GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOB->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);
    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0);
    GPIOB->AFR[1] |= (4 << 0) | (4 << 4);
    I2C1->TIMINGR = 0x00303D5B;
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_ReadMulti(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint8_t len) {
    I2C1->CR2 = (devAddr & I2C_CR2_SADD) | (1 << 16) | I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = regAddr;
    while (!(I2C1->ISR & I2C_ISR_TC));
    I2C1->CR2 = (devAddr & I2C_CR2_SADD) | ((uint32_t)len << 16) | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;
    for (uint8_t i = 0; i < len; i++) {
        while (!(I2C1->ISR & I2C_ISR_RXNE));
        buf[i] = (uint8_t)I2C1->RXDR;
    }
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

void IMU_SetBank(uint8_t bank) { IMU_WriteReg(0x7F, (bank & 0x03) << 4); }

void IMU_Init(void) {
    // Пробуждение...
    IMU_SetBank(0);
    IMU_WriteReg(0x06, 0x01);
    for(volatile int i=0; i<100000; i++);
    IMU_WriteReg(0x07, 0x00);
    
    IMU_SetBank(2);
    IMU_WriteReg(0x01, 0x01); // Bypass (отключаем встроенный фильтр для анализа)
    IMU_SetBank(0);

    // Начальная инициализация ( на 0 Гц dsp_manager сам их включит)
    biquad_init_notch(&notch1, 0, 1.0f, 1000.0f);
    biquad_init_notch(&notch2, 0, 1.0f, 1000.0f);
    biquad_init_notch(&notch3, 0, 1.0f, 1000.0f);
}

void IMU_Calibrate(void) {
    int32_t gx_s = 0; uint8_t buf[14];
    for (int i = 0; i < 512; i++) {
        I2C_ReadMulti(IMU_ADDR, 0x2D, buf, 14);
        gx_s += (int16_t)(buf[6] << 8 | buf[7]);
        for (volatile int d = 0; d < 2000; d++);
    }
    gyro_bias_x = (float)gx_s / 512.0f;
}

void IMU_ReadRawData(void) {
    uint8_t buf[14];
    I2C_ReadMulti(IMU_ADDR, 0x2D, buf, 14);
    
    raw_gx = (int16_t)(buf[6] << 8 | buf[7]);
    float x = (float)raw_gx - gyro_bias_x;

    // Последовательно применяем 3 режекторных фильтра
    // dsp_manager будет менять их коэффициенты в фоновом режиме
    x = biquad_apply(&notch1, x);
    x = biquad_apply(&notch2, x);
    x = biquad_apply(&notch3, x);

    filt_gx = x;
}
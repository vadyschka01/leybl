#include "imu.h"
#include <math.h>
#include "stm32g4xx.h"
#include "stm32g431xx.h"

#ifndef FMAC_PARAM_FUNC_Pos
#define FMAC_PARAM_FUNC_Pos   (0U)
#define FMAC_PARAM_P_Pos      (8U)
#define FMAC_PARAM_Q_Pos      (16U)
#define FMAC_PARAM_RSHIFT_Pos (24U)
#endif

#ifndef FMAC_SR_VLD
#define FMAC_SR_VLD           (1U << 0)

#endif


// Константы смещения в памяти FMAC (всего 256 слов)
// Каждая Notch-секция (IIR 2-го порядка) требует:
// 3 коэфф. B, 2 коэфф. A, 2 ячейки истории X, 2 ячейки истории Y.
#define FMAC_MEM_SIZE      256
#define STAGE_SIZE         10  // Резервируем с запасом под каждый каскад

// raw_ax, raw_ay, raw_az удалены (не используются)
volatile int16_t raw_gx; // Нужен только для гироскопа X
float filt_gx;
float gyro_bias_x = 0;

// notch1, notch2, notch3 удалены (заменены на notch_fmac_coeffs[3])
// biquad_apply и biquad_init_notch удалены (больше не нужны с FMAC)

fmac_coeffs_t notch_fmac_coeffs[3];
fmac_state_t  notch_fmac_state[3];

// 1. Инициализация (с правильной разметкой памяти)
void FMAC_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_FMACEN;
    RCC->AHB1RSTR |= RCC_AHB1RSTR_FMACRST;
    for(volatile int i=0; i<100; i++);
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_FMACRST;

    // Конфигурация памяти: X1 (коэф), X2 (входы), Y (выходы)
    FMAC->X1BUFCFG = (5 << 8) | (0 << 0); // 5 коэф. с адреса 0
    FMAC->X2BUFCFG = (2 << 8) | (5 << 0); // 2 входа с адреса 5
    FMAC->YBUFCFG  = (2 << 8) | (7 << 0); // 2 выхода с адреса 7

    FMAC->CR = 0x01; // Включаем модуль
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

    // b0 = 1.0 (в Q14 это 16384), остальные 0
    for (int i = 0; i < 3; i++) {
    notch_fmac_coeffs[i].b0 = 0; 
    notch_fmac_coeffs[i].b1 = 0;
    notch_fmac_coeffs[i].b2 = 0;
    notch_fmac_coeffs[i].a1 = 0;
    notch_fmac_coeffs[i].a2 = 0;
    
    notch_fmac_state[i].x1 = 0; 
    notch_fmac_state[i].x2 = 0;
    notch_fmac_state[i].y1 = 0; 
    notch_fmac_state[i].y2 = 0;
}
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

    // ВМЕСТО ЭТОГО:
    // x = biquad_apply(&notch1, x);
    // x = biquad_apply(&notch2, x);
    // x = biquad_apply(&notch3, x);

    // ТЕПЕРЬ:
    x = FMAC_Process_Sample(x);

    filt_gx = x;
}

void Update_FMAC_Coeffs(int stage, float b0, float b1, float b2, float a1, float a2) {
    if (stage < 0 || stage > 2) return;
    const float scale = 16384.0f; // Q14
    
    notch_fmac_coeffs[stage].b0 = (int16_t)(b0 * scale);
    notch_fmac_coeffs[stage].b1 = (int16_t)(b1 * scale);
    notch_fmac_coeffs[stage].b2 = (int16_t)(b2 * scale);
    // Для FMAC знаки a1 и a2 инвертируем!
    notch_fmac_coeffs[stage].a1 = (int16_t)(-a1 * scale);
    notch_fmac_coeffs[stage].a2 = (int16_t)(-a2 * scale);
}

// Внутренняя функция для обработки одного каскада через FMAC
// 2. Шаг вычислений (с защитой от зависания и обнуления)
static int16_t FMAC_Step(fmac_coeffs_t *c, fmac_state_t *s, int16_t input) {
    // Если фильтр в режиме Bypass (b0=16384, b1=0), просто возвращаем вход
    if (c->b0 == 16384 && c->b1 == 0) return input;

    // Сброс FIFO перед каждой операцией (критично для Polling режима)
    FMAC->CR &= ~0x01; 
    FMAC->CR |= 0x01;

    // Пишем коэффы (5 штук)
    FMAC->WDATA = c->b0; FMAC->WDATA = c->b1; FMAC->WDATA = c->b2;
    FMAC->WDATA = c->a1; FMAC->WDATA = c->a2;
    
    // Пишем историю (4 штуки)
    FMAC->WDATA = s->x1; FMAC->WDATA = s->x2;
    FMAC->WDATA = s->y1; FMAC->WDATA = s->y2;

    // Настройка: FUNC=8 (IIR), P=3, Q=2, RSHIFT=1 (бит 24)
    // RSHIFT=1 компенсирует масштаб 16384
    FMAC->PARAM = (1U << 24) | (2U << 16) | (3U << 8) | (8 << 0);
    
    FMAC->WDATA = input;
    
    uint32_t timeout = 1000;
    while (!(FMAC->SR & 0x01) && --timeout);
    
    if (timeout == 0) return input; 

    int16_t result = (int16_t)FMAC->RDATA;

    // Если FMAC выдал ровно 0 при живом входе - это ошибка, возвращаем вход
    if (result == 0 && input != 0) return input;

    // Сохраняем состояние
    s->x2 = s->x1; s->x1 = input;
    s->y2 = s->y1; s->y1 = result;

    return result;
}

// 3. Главная точка входа
float FMAC_Process_Sample(float input) {
    int16_t val = (int16_t)input;
    
    val = FMAC_Step(&notch_fmac_coeffs[0], &notch_fmac_state[0], val);
    val = FMAC_Step(&notch_fmac_coeffs[1], &notch_fmac_state[1], val);
    val = FMAC_Step(&notch_fmac_coeffs[2], &notch_fmac_state[2], val);
    
    return (float)val;
}

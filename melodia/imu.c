#include "stm32g4xx.h"
#include "imu.h"

volatile uint8_t  imu_whoami = 0;
volatile int16_t  imu_ax = 0;
volatile int16_t  imu_ay = 0;
volatile int16_t  imu_az = 0;
volatile int16_t  imu_gx = 0;
volatile int16_t  imu_gy = 0;
volatile int16_t  imu_gz = 0;
volatile int16_t  imu_temp_raw = 0;
volatile float    imu_temp_c = 0.0f;

static void delay_long(int limit) {
    for (volatile int i = 0; i < limit; i++) __NOP();
}

void I2C1_Init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

    GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOB->MODER |=  (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);
    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
    GPIOB->PUPDR  |= (GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0);
    GPIOB->AFR[1] |= (4 << 0) | (4 << 4);

    I2C1->TIMINGR = 0x00303D5B;
    I2C1->CR1 |= I2C_CR1_PE;
}

uint8_t I2C_ReadReg(uint8_t devAddr, uint8_t regAddr) {
    I2C1->CR2 = (devAddr & I2C_CR2_SADD) | (1 << 16) | I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXIS)) {
        if (I2C1->ISR & I2C_ISR_NACKF) return 0xFF;
    }
    I2C1->TXDR = regAddr;
    while (!(I2C1->ISR & I2C_ISR_TC));

    I2C1->CR2 = (devAddr & I2C_CR2_SADD) | (1 << 16) |
                I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;

    while (!(I2C1->ISR & I2C_ISR_RXNE));
    return (uint8_t)I2C1->RXDR;
}

void I2C_ReadMulti(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint8_t len) {
    I2C1->CR2 = (devAddr & I2C_CR2_SADD) | (1 << 16) | I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXIS)) {
        if (I2C1->ISR & I2C_ISR_NACKF) return;
    }
    I2C1->TXDR = regAddr;
    while (!(I2C1->ISR & I2C_ISR_TC));

    I2C1->CR2 = (devAddr & I2C_CR2_SADD) | ((uint32_t)len << 16) |
                I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;

    for (uint8_t i = 0; i < len; i++) {
        while (!(I2C1->ISR & I2C_ISR_RXNE));
        buf[i] = (uint8_t)I2C1->RXDR;
    }
}

static void IMU_WriteReg(uint8_t reg, uint8_t val) {
    I2C1->CR2 = (IMU_ADDR & I2C_CR2_SADD) | (2 << 16) | I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXIS)) {
        if (I2C1->ISR & I2C_ISR_NACKF) return;
    }
    I2C1->TXDR = reg;
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = val;
    while (!(I2C1->ISR & I2C_ISR_TC));
    I2C1->CR2 |= I2C_CR2_STOP;
}

void IMU_SetBank(uint8_t bank) {
    IMU_WriteReg(0x7F, (uint8_t)(bank << 4));
}

void IMU_Init(void) {
    IMU_SetBank(0);
    IMU_WriteReg(0x06, 0x80);
    delay_long(100000);
    IMU_WriteReg(0x06, 0x01);
    delay_long(100000);
}

void IMU_ReadAccelGyro(void) {
    uint8_t buf[14];

    IMU_SetBank(0);
    I2C_ReadMulti(IMU_ADDR, REG_ACCEL_GYRO_START, buf, 14);

    imu_ax = (int16_t)((buf[0] << 8) | buf[1]);
    imu_ay = (int16_t)((buf[2] << 8) | buf[3]);
    imu_az = (int16_t)((buf[4] << 8) | buf[5]);
    imu_gx = (int16_t)((buf[6] << 8) | buf[7]);
    imu_gy = (int16_t)((buf[8] << 8) | buf[9]);
    imu_gz = (int16_t)((buf[10] << 8) | buf[11]);

    imu_temp_raw = (int16_t)((buf[12] << 8) | buf[13]);
    imu_temp_c = 21.0f + ((float)imu_temp_raw) / 333.87f;
}

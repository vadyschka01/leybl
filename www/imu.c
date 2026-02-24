#include "stm32g4xx.h"
#include "imu.h"
#include <math.h>

// ---------------- RAW IMU DATA ----------------
volatile int16_t imu_ax = 0;
volatile int16_t imu_ay = 0;
volatile int16_t imu_az = 0;

// ---------------- NORMALIZED ----------------
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;

// ---------------- gyro ----------------
float gyro_roll_rate  = 0.0f;
float gyro_pitch_rate = 0.0f;
float gyro_yaw_rate   = 0.0f;

volatile int16_t imu_gx = 0;
volatile int16_t imu_gy = 0;
volatile int16_t imu_gz = 0;


// ---------------- bias ----------------
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;



// ---------------- ANGLES ----------------
float roll_acc  = 0.0f;
float pitch_acc = 0.0f;


float roll_angle  = 0.0f;
float pitch_angle = 0.0f;

// ---------------- I2C + IMU INIT ----------------
static void delay_long(int limit) {
    for (volatile int i = 0; i < limit; i++) __NOP();
}

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

void I2C_ReadMulti(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint8_t len) {
    I2C1->CR2 = (devAddr & I2C_CR2_SADD) | (1 << 16) | I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXIS));
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
    while (!(I2C1->ISR & I2C_ISR_TXIS));
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
    IMU_WriteReg(0x06, 0x80); // reset
    delay_long(100000);
    IMU_WriteReg(0x06, 0x01); // clock
    delay_long(100000);
}


// ---------------- calibrateGyro----------------
void IMU_CalibrateGyro(void) {
    const int samples = 500;
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;

    for (int i = 0; i < samples; i++) {
        uint8_t buf[14];
        IMU_SetBank(0);
        I2C_ReadMulti(IMU_ADDR, REG_ACCEL_GYRO_START, buf, 14);

        int16_t gx = (int16_t)((buf[6] << 8) | buf[7]);
        int16_t gy = (int16_t)((buf[8] << 8) | buf[9]);
        int16_t gz = (int16_t)((buf[10] << 8) | buf[11]);

        sum_x += gx;
        sum_y += gy;
        sum_z += gz;

        for (volatile int d = 0; d < 3000; d++); // маленькая задержка
    }

    gyro_bias_x = (float)sum_x / samples;
    gyro_bias_y = (float)sum_y / samples;
    gyro_bias_z = (float)sum_z / samples;
}



// ---------------- READ ACCEL ONLY ----------------
void IMU_ReadAccel(void) {
    uint8_t buf[14];

    IMU_SetBank(0);
    I2C_ReadMulti(IMU_ADDR, REG_ACCEL_GYRO_START, buf, 14);

    // --- ACCEL ---
    imu_ax = (int16_t)((buf[0] << 8) | buf[1]);
    imu_ay = (int16_t)((buf[2] << 8) | buf[3]);
    imu_az = (int16_t)((buf[4] << 8) | buf[5]);

    accel_x = (float)imu_ax / 16384.0f;
    accel_y = (float)imu_ay / 16384.0f;
    accel_z = (float)imu_az / 16384.0f;

    pitch_acc = atan2f(accel_y, accel_z) * 57.2958f;
    roll_acc  = atan2f(-accel_x, sqrtf(accel_y*accel_y + accel_z*accel_z)) * 57.2958f;

    // --- GYRO RAW ---
    imu_gx = (int16_t)((buf[6] << 8) | buf[7]);
    imu_gy = (int16_t)((buf[8] << 8) | buf[9]);
    imu_gz = (int16_t)((buf[10] << 8) | buf[11]);
    
    // ---GYRO BIAS
    float gx = imu_gx - gyro_bias_x;
    float gy = imu_gy - gyro_bias_y;
    float gz = imu_gz - gyro_bias_z;


    // --- GYRO DEG/SEC +BIAS---
    const float scale = 2000.0f / 32768.0f;

    gyro_roll_rate  = gy * scale;  // <-- теперь roll сидит на gy
    gyro_pitch_rate = gx * scale;  // <-- а pitch на gx
    gyro_yaw_rate   = gz * scale;

    
    // === INTEGRATE GYRO ===
    float dt = 0.02f;   // если IMU вызывается каждые 20 мс (50 Гц)
    roll_angle  += gyro_roll_rate  * dt;
    pitch_angle += gyro_pitch_rate * dt;

    
    // === COMPLEMENTARY FILTER ===
    float alpha = 0.995f;  // можно будет подстроить

    roll_angle  = alpha * roll_angle  + (1.0f - alpha) * roll_acc;
    pitch_angle = alpha * pitch_angle + (1.0f - alpha) * pitch_acc;


}


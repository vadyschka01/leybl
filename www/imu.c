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


// ----------- ACCEL OFFSETS -----------
float accel_offset_x = 0.0f;
float accel_offset_y = 0.0f;
float accel_offset_z = 0.0f;


// ---------------- gyro ----------------
float gyro_roll_rate  = 0.0f;
float gyro_pitch_rate = 0.0f;
float gyro_yaw_rate   = 0.0f;

float pitch_gyro = 0.0f;
float roll_gyro  = 0.0f;


volatile int16_t imu_gx = 0;
volatile int16_t imu_gy = 0;
volatile int16_t imu_gz = 0;

//----------биквад
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float d1, d2;
} biquad_t;

static biquad_t accel_x_f, accel_y_f, accel_z_f;
static biquad_t gyro_x_f,  gyro_y_f,  gyro_z_f;


// ---------------- bias ----------------
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;

//Правильный dt
uint32_t last_time = 0;


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
    uint8_t val = (bank & 0x03) << 4;   // 0, 0x10, 0x20, 0x30
    IMU_WriteReg(0x7F, val);
}

//---------------биквад
static float biquad_apply(biquad_t *f, float x) {
    float result = f->b0 * x + f->d1;
    f->d1 = f->b1 * x - f->a1 * result + f->d2;
    f->d2 = f->b2 * x - f->a2 * result;
    return result;
}

static void biquad_init_lpf(biquad_t *f, float cutoff, float sample_rate) {
    float w0 = 2.0f * 3.1415926f * cutoff / sample_rate;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float Q = 0.707f;

    float alpha = sinw0 / (2.0f * Q);

    float b0 = (1 - cosw0) * 0.5f;
    float b1 = 1 - cosw0;
    float b2 = (1 - cosw0) * 0.5f;
    float a0 = 1 + alpha;
    float a1 = -2 * cosw0;
    float a2 = 1 - alpha;

    f->b0 = b0 / a0;
    f->b1 = b1 / a0;
    f->b2 = b2 / a0;
    f->a1 = a1 / a0;
    f->a2 = a2 / a0;

    f->d1 = 0;
    f->d2 = 0;
}


void IMU_Init(void) {
    // BANK 0
    IMU_SetBank(0);

    // Reset
    IMU_WriteReg(0x06, 0x80);   // PWR_MGMT_1: DEVICE_RESET = 1
    delay_long(100000);

    // Выход из sleep + выбор тактирования (CLKSEL = 1, SLEEP = 0)
    IMU_WriteReg(0x06, 0x01);
    delay_long(100000);

    // Включить аксель и гиру (PWR_MGMT_2 = 0x00)
    IMU_WriteReg(0x07, 0x00);

    // BANK 2 — настройки диапазонов и фильтров
    IMU_SetBank(2);

    // GYRO_CONFIG_1 (адрес 0x01 в BANK 2)
    // GYRO_FS_SEL = 3 (±2000 dps), DLPF включен, частота по умолчанию
    IMU_WriteReg(0x01, 0x07);   // 0b0000 0111

    // ACCEL_CONFIG (адрес 0x14 в BANK 2)
    // ACCEL_FS_SEL = 0 (±2g, 16384 LSB/g — как ты используешь)
    IMU_WriteReg(0x14, 0x00);

    // Вернуться в BANK 0 для чтения данных
    IMU_SetBank(0);
    
    // === BIQUAD FILTER INIT ===
    float fs = 100.0f;  // частота вызова IMU_ReadAccel()

    // ГИРО — LPF 80 Гц
    biquad_init_lpf(&gyro_x_f, 25.0f, fs);
    biquad_init_lpf(&gyro_y_f, 25.0f, fs);
    biquad_init_lpf(&gyro_z_f, 25.0f, fs);

    // АКСЕЛЬ — LPF 30 Гц
    biquad_init_lpf(&accel_x_f, 25.0f, fs);
    biquad_init_lpf(&accel_y_f, 25.0f, fs);
    biquad_init_lpf(&accel_z_f, 25.0f, fs);
}


void IMU_CalibrateAccel(void) {
    const int samples = 500;
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;

    for (int i = 0; i < samples; i++) {
        uint8_t buf[14];
        IMU_SetBank(0);
        I2C_ReadMulti(IMU_ADDR, REG_ACCEL_GYRO_START, buf, 14);

        int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t az = (int16_t)((buf[4] << 8) | buf[5]);

        sum_x += (float)ax / 16384.0f;
        sum_y += (float)ay / 16384.0f;
        sum_z += (float)az / 16384.0f;

        for (volatile int d = 0; d < 3000; d++);
    }

    accel_offset_x = sum_x / samples;
    accel_offset_y = sum_y / samples;
    accel_offset_z = (sum_z / samples) - 1.0f; // Z должен быть +1g
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
    
    static long test_count=0;
    test_count++;
    
    float dt = 0.01f;  // ----100ГЦ
   
    IMU_SetBank(0);
    I2C_ReadMulti(IMU_ADDR, REG_ACCEL_GYRO_START, buf, 14);

    // --- ACCEL ---
    imu_ax = (int16_t)((buf[0] << 8) | buf[1]);
    imu_ay = (int16_t)((buf[2] << 8) | buf[3]);
    imu_az = (int16_t)((buf[4] << 8) | buf[5]);

    accel_x = (float)imu_ax / 16384.0f - accel_offset_x;
    accel_y = (float)imu_ay / 16384.0f - accel_offset_y;
    accel_z = (float)imu_az / 16384.0f - accel_offset_z;
    
    //биквад аксель
    accel_x = biquad_apply(&accel_x_f, accel_x);
    accel_y = biquad_apply(&accel_y_f, accel_y);
    accel_z = biquad_apply(&accel_z_f, accel_z);


    roll_acc  = -atan2f(accel_x, accel_z) * 57.2958f;
    pitch_acc = -atan2f(accel_y,sqrtf(accel_x * accel_x + accel_z * accel_z)) * 57.2958f;


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

    gyro_roll_rate  = gy * scale;  // 
    gyro_pitch_rate = -gx * scale;  // 
    gyro_yaw_rate   = gz * scale;
    
        //биквад гиро
    gx = biquad_apply(&gyro_x_f, gx);
    gy = biquad_apply(&gyro_y_f, gy);
    gz = biquad_apply(&gyro_z_f, gz);


    
    // === INTEGRATE GYRO ===
    roll_gyro  += gyro_roll_rate  * dt;
    pitch_gyro += gyro_pitch_rate * dt;

    if (roll_gyro > 60.0f)  roll_gyro = 60.0f;
    if (roll_gyro < -60.0f) roll_gyro = -60.0f;

    if (pitch_gyro > 60.0f)  pitch_gyro = 60.0f;
    if (pitch_gyro < -60.0f) pitch_gyro = -60.0f;


    
    // === COMPLEMENTARY FILTER ===
    float alpha = 0.995f;

    roll_angle  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;
    pitch_angle = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
    
    roll_gyro = roll_angle;
    pitch_gyro = pitch_angle;



}


#include "stm32g4xx.h"
#include "imu.h"
#include <math.h>

// ---------------- RAW IMU DATA ----------------
volatile uint8_t  imu_whoami = 0;
volatile int16_t  imu_ax = 0;
volatile int16_t  imu_ay = 0;
volatile int16_t  imu_az = 0;
volatile int16_t  imu_gx = 0;
volatile int16_t  imu_gy = 0;
volatile int16_t  imu_gz = 0;
volatile int16_t  imu_temp_raw = 0;
volatile float    imu_temp_c = 0.0f;

// ---------------- FILTERED / PROCESSED DATA ----------------
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;

float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;

float gyro_roll_rate  = 0.0f;
float gyro_pitch_rate = 0.0f;
float gyro_yaw_rate   = 0.0f;

// ---------------- GYRO BIAS ----------------
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;

// ---------------- ANGLES ----------------
float roll_angle = 0;
float pitch_angle = 0;

float pitch_trim_deg = 0.0f;

// ---------------- BIQUAD FILTER ----------------
typedef struct {
    float b0, b1, b2, a1, a2;
    float d1, d2;
} biquad_t;

static biquad_t accel_x_lpf;
static biquad_t accel_y_lpf;
static biquad_t accel_z_lpf;

static void biquad_init(biquad_t *f, float cutoff, float sample_rate) {
    float omega = 2.0f * 3.1415926f * cutoff / sample_rate;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * 0.707f);

    float b0 = (1 - cs) * 0.5f;
    float b1 = 1 - cs;
    float b2 = (1 - cs) * 0.5f;
    float a0 = 1 + alpha;
    float a1 = -2 * cs;
    float a2 = 1 - alpha;

    f->b0 = b0 / a0;
    f->b1 = b1 / a0;
    f->b2 = b2 / a0;
    f->a1 = a1 / a0;
    f->a2 = a2 / a0;

    f->d1 = 0;
    f->d2 = 0;
}

static float biquad_apply(biquad_t *f, float x) {
    float result = f->b0 * x + f->d1;
    f->d1 = f->b1 * x - f->a1 * result + f->d2;
    f->d2 = f->b2 * x - f->a2 * result;
    return result;
}

// ---------------- I2C + IMU INIT ----------------
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

    biquad_init(&accel_x_lpf, 30.0f, 1000.0f);
    biquad_init(&accel_y_lpf, 30.0f, 1000.0f);
    biquad_init(&accel_z_lpf, 30.0f, 1000.0f);
}

// ---------------- GYRO CALIBRATION ----------------
void IMU_CalibrateGyro(void) {
    const int N = 1000;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < N; i++) {
        uint8_t buf[14];
        IMU_SetBank(0);
        I2C_ReadMulti(IMU_ADDR, REG_ACCEL_GYRO_START, buf, 14);

        int16_t gx = (int16_t)((buf[6] << 8) | buf[7]);
        int16_t gy = (int16_t)((buf[8] << 8) | buf[9]);
        int16_t gz = (int16_t)((buf[10] << 8) | buf[11]);

        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
    }

    gyro_bias_x = (float)sum_x / N;
    gyro_bias_y = (float)sum_y / N;
    gyro_bias_z = (float)sum_z / N;
}

// ---------------- MAIN IMU READ ----------------
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

    accel_x = biquad_apply(&accel_x_lpf, imu_ax);
    accel_y = biquad_apply(&accel_y_lpf, imu_ay);
    accel_z = biquad_apply(&accel_z_lpf, imu_az);

    imu_temp_raw = (int16_t)((buf[12] << 8) | buf[13]);
    imu_temp_c = 21.0f + ((float)imu_temp_raw) / 333.87f;

    float ax = accel_x / 16384.0f;
    float ay = accel_y / 16384.0f;
    float az = accel_z / 16384.0f;

    float roll_acc  = atan2f(ax, az) * 57.2958f;
    float pitch_acc = atan2f(-ay, sqrtf(ax*ax + az*az)) * 57.2958f;
    pitch_acc += pitch_trim_deg;

    // -------- APPLY BIAS --------
    float gx_raw = (float)imu_gx - gyro_bias_x;
    float gy_raw = (float)imu_gy - gyro_bias_y;
    float gz_raw = (float)imu_gz - gyro_bias_z;

    // -------- CONVERT TO DEG/SEC --------
    const float scale = (2000.0f / 32768.0f);

    gyro_roll_rate  = gx_raw * scale;
    gyro_pitch_rate = gy_raw * scale;
    gyro_yaw_rate   = gz_raw * scale;

    // -------- INTEGRATE --------
    float dt = 0.001f;

    roll_angle  += gyro_roll_rate  * dt;
    pitch_angle += gyro_pitch_rate * dt;

    // -------- COMPLEMENTARY FILTER --------
    float alpha = 0.997f;

    roll_angle  = roll_angle  * alpha + roll_acc  * (1.0f - alpha);
    pitch_angle = pitch_angle * alpha + pitch_acc * (1.0f - alpha);
}

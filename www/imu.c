#include "stm32g4xx.h"
#include "imu.h"
#include <math.h>


volatile uint8_t  imu_whoami = 0;
volatile int16_t  imu_ax = 0;
volatile int16_t  imu_ay = 0;
volatile int16_t  imu_az = 0;
volatile int16_t  imu_gx = 0;
volatile int16_t  imu_gy = 0;
volatile int16_t  imu_gz = 0;
volatile int16_t  imu_temp_raw = 0;
volatile float    imu_temp_c = 0.0f;



float gyro_x = 0;  // --- для pidov
float gyro_y = 0;
float gyro_z = 0;

float accel_x = 0;
float accel_y = 0;
float accel_z = 0;  // ---


typedef struct {                                                                //----
    float b0, b1, b2, a1, a2;
    float d1, d2;
} biquad_t;

static void biquad_init(biquad_t *f, float cutoff, float sample_rate) {         
    float omega = 2.0f * 3.1415926f * cutoff / sample_rate;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * 0.707f); // Q = 0.707 (Butterworth)

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
}                                                                               // ----

static biquad_t accel_x_lpf; // для biquad фильтрации
static biquad_t accel_y_lpf; 
static biquad_t accel_z_lpf;

static float biquad_apply(biquad_t *f, float x) {
    float result = f->b0 * x + f->d1;
    f->d1 = f->b1 * x - f->a1 * result + f->d2;
    f->d2 = f->b2 * x - f->a2 * result;
    return result;
}


float pitch_angle = 0;  // angle
float roll_angle = 0;




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
    
    biquad_init(&accel_x_lpf, 30.0f, 1000.0f); // cutoff 30 Hz, sample rate 1000 Hz
    biquad_init(&accel_y_lpf, 30.0f, 1000.0f);
    biquad_init(&accel_z_lpf, 30.0f, 1000.0f);

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
    
    
    // === Пробрасываем в PID-переменные ===
    gyro_x = imu_gx;
    gyro_y = imu_gy;
    gyro_z = imu_gz;

    accel_x = imu_ax;
    accel_y = imu_ay;
    accel_z = imu_az;  // --- для pidov
    
    float ax = biquad_apply(&accel_x_lpf, accel_x);
    float ay = biquad_apply(&accel_y_lpf, accel_y);
    float az = biquad_apply(&accel_z_lpf, accel_z);




    imu_temp_raw = (int16_t)((buf[12] << 8) | buf[13]);
    imu_temp_c = 21.0f + ((float)imu_temp_raw) / 333.87f;
    
    
    // нормализация акселя
    ax /= 16384.0f;
    ay /= 16384.0f;
    az /= 16384.0f;

    // правильные углы из акселя
    float roll_acc  = atan2f(ay, az) * 57.2958f;
    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2958f;

    // перевод гиры в градусы/секунду
    float gx = gyro_x * (2000.0f / 32768.0f);   // roll rate
    float gy = gyro_y * (2000.0f / 32768.0f);   // pitch rate

    // dt
    float dt = 0.001f;

    // интеграция гиры
    roll_angle  += gx * dt;   // <-- было наоборот
    pitch_angle += gy * dt;   // <-- было наоборот

    // комплементарный фильтр
    float alpha = 0.997f;

    roll_angle  = roll_angle  * alpha + roll_acc  * (1.0f - alpha);
    pitch_angle = pitch_angle * alpha + pitch_acc * (1.0f - alpha);



}

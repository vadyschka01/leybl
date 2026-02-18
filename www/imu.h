#ifndef IMU_H
#define IMU_H

#include <stdint.h>
extern float gyro_x;    //--- для pidov
extern float gyro_y;
extern float gyro_z;

extern float accel_x;
extern float accel_y;
extern float accel_z;

extern float pitch_angle;  // --- angle 
extern float roll_angle;



#define IMU_ADDR      (0x68 << 1)
#define REG_WHO_AM_I  0x00
#define REG_ACCEL_GYRO_START 0x2D

// Глобальные переменные IMU (extern!)
extern volatile uint8_t  imu_whoami;
extern volatile int16_t  imu_ax;
extern volatile int16_t  imu_ay;
extern volatile int16_t  imu_az;
extern volatile int16_t  imu_gx;
extern volatile int16_t  imu_gy;
extern volatile int16_t  imu_gz;
extern volatile int16_t  imu_temp_raw;
extern volatile float    imu_temp_c;

// Функции
void I2C1_Init(void);
uint8_t I2C_ReadReg(uint8_t devAddr, uint8_t regAddr);
void I2C_ReadMulti(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint8_t len);

void IMU_Init(void);
void IMU_SetBank(uint8_t bank);
void IMU_ReadAccelGyro(void);

#endif

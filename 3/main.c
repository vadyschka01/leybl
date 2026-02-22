#include "stm32g4xx.h"
#include "imu.h"
#include "sbus.h"
#include "motors.h"

volatile uint32_t ms = 0;
uint8_t armed = 0;

void SysTick_Handler(void) {
    ms++;
}

int main(void) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    SysTick_Config(SystemCoreClock / 1000U);

    // === 1. СНАЧАЛА МОТОРЫ ===
    Motors_Init();
    Set_Motors(900); // минимальный сигнал ESC
    for (volatile int i = 0; i < 6000000; i++) __NOP(); // 3 секунды

    // === 2. ПОТОМ IMU ===
    I2C1_Init();
    IMU_Init();
    
    // === 3. ПОТОМ SBUS ===
    LPUART1_SBUS_Init();
    
    PID_Init();    // pid иниц

    uint32_t last_imu = 0;

    while (1) {

        // === Чтение IMU ===
        if (ms - last_imu >= 20) {
            IMU_ReadAccelGyro();
            last_imu = ms;
        }

        // === ARM / DISARM через SWA ===
        int swa = rc_channels[4]; // канал тумблера

        if (!armed) {
            if (swa > 1500 && rc_channels[2] < 1050) {
                armed = 1;
            }
        } else {
            if (swa < 1500) {
                armed = 0;
                Set_Motors(900);
            }
        }

        if (armed) {
            PID_Update();
            Mixer_Update();
        } else {
            Motors_Stop();
        }


    }
}

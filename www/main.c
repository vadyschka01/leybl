/* #include "stm32g4xx.h"
#include "imu.h"
#include "sbus.h"
#include "motors.h"
#include "pid.h"

volatile uint32_t ms = 0;
volatile uint32_t tim6_count = 0;

volatile uint8_t armed = 0;

// ===== SysTick 1 kHz =====
void SysTick_Handler(void) {
    ms++;
}

// ===== TIM6 100 Hz =====
void TIM6_Init_100Hz(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;

    TIM6->PSC = 16000 - 1;   // 160 MHz /
    TIM6->ARR = 100 - 1;     // 1000 Hz / 10 = 100 Hz

    TIM6->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    TIM6->CR1 |= TIM_CR1_CEN;
}

// ===== TIM6 IRQ — основной цикл управления =====
volatile uint8_t imu_flag = 0;

void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR = 0;
        imu_flag = 1;   // только флаг
        
        
        
    }
}


int main(void) {

    // === CLOCK ===
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    SysTick_Config(SystemCoreClock / 1000U);   // 1 kHz

    // === 1. МОТОРЫ ===
    // Motors_Init();
    Set_Motors(900); // минимальный сигнал ESC
    for (volatile int i = 0; i < 6000000; i++) __NOP(); // 3 секунды

    // === 2. IMU ===
    I2C1_Init();
    IMU_Init();
    IMU_CalibrateAccel();
    IMU_CalibrateGyro();

    roll_angle  = 0.0f;
    pitch_angle = 0.0f;

    // === 3. SBUS ===
    LPUART1_SBUS_Init();

    PID_Init();
    TIM6_Init_100Hz();

    // ===== MAIN LOOP =====
   while (1) {

    // ARM / DISARM
    int swa = rc_channels[4];

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

    // === ОСНОВНОЙ ЦИКЛ УПРАВЛЕНИЯ ===
    if (imu_flag) {
        imu_flag = 0;

        IMU_ReadAccel();   // тяжёлая функция
        if (armed) {
            PID_Update();
            Mixer_Update();
        } else {
            Motors_Stop();
        }
    }
   }
} */



#include "stm32g4xx.h"
#include "imu.h"
#include "sbus.h"
#include "motors.h"
#include "pid.h"

volatile uint32_t ms = 0;
volatile uint8_t armed = 0;

// ===== SysTick 1 kHz =====
void SysTick_Handler(void) {
    ms++;
}

// ===== TIM6 100 Hz =====
void TIM6_Init_100Hz(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;

    TIM6->PSC = 16000 - 1;   // 160 MHz / 16000 = 10 kHz
    TIM6->ARR = 10 - 1;     // 10 kHz / 100 = 100 Hz

    TIM6->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM6_DAC_IRQn, 1);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    TIM6->CR1 |= TIM_CR1_CEN;
}

// ===== TIM6 IRQ — основной цикл управления =====
void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR = 0;
        
        static long test_count_2=0;
        // === IMU ===
        IMU_ReadAccel();   // dt = 0.01f внутри IMU.c
        
        test_count_2++;
        // === PID + MIXER ===
        if (armed) {
            PID_Update();
            Mixer_Update();
        } else {
            Motors_Stop();
        }
    }
}

int main(void) {

    // === CLOCK ===
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    SysTick_Config(SystemCoreClock / 1000U);   // 1 kHz

    // === МОТОРЫ ===
    Motors_Init();
    Set_Motors(900);
    for (volatile int i = 0; i < 6000000; i++) __NOP();

    // === IMU ===
    I2C1_Init();
    IMU_Init();
    IMU_CalibrateAccel();
    IMU_CalibrateGyro();

    roll_angle  = 0.0f;
    pitch_angle = 0.0f;

    // === SBUS ===
    LPUART1_SBUS_Init();

    PID_Init();
    TIM6_Init_100Hz();

    // ===== MAIN LOOP =====
    while (1) {

        int swa = rc_channels[4];

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
    }
}

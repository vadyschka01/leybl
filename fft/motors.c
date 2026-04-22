#include "stm32g4xx.h"
#include "motors.h"
#include "pid.h"
#include "mixer.h"

void Motors_Init(void) {

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // PA0, PA1 -> TIM2 CH1/CH2
    GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
    GPIOA->MODER |=  (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1);
    GPIOA->AFR[0] &= ~((0xF << 0) | (0xF << 4));
    GPIOA->AFR[0] |=  ((1 << 0) | (1 << 4));

    // PA10, PA11 -> TIM1 CH3/CH4
    GPIOA->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11);
    GPIOA->MODER |=  (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1);
    GPIOA->AFR[1] &= ~((0xF << 8) | (0xF << 12));
    GPIOA->AFR[1] |=  ((6 << 8) | (11 << 12));

    // TIM2 — 100 Гц
    TIM2->PSC = 16 - 1;
    TIM2->ARR = 10000 - 1;
    TIM2->CCMR1 = 0x6060;
    TIM2->CCER  = 0x11;
    TIM2->CR1   = TIM_CR1_CEN;

    // TIM1 — 100 Гц
    TIM1->PSC = 16 - 1;
    TIM1->ARR = 10000 - 1;
    TIM1->CCMR2 = 0x6060;
    TIM1->CCER  = 0x1100;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1   = TIM_CR1_CEN;
}

void Set_Motors(int val) {
    TIM2->CCR1 = val; // M1
    TIM2->CCR2 = val; // M2
    TIM1->CCR3 = val; // M3
    TIM1->CCR4 = val; // M4
}

void Set_Motor_Individual(int m1, int m2, int m3, int m4) {
    TIM2->CCR1 = m1;
    TIM2->CCR2 = m2;
    TIM1->CCR3 = m3;
    TIM1->CCR4 = m4;
}

void Motors_StartupDelay(void) {
    Set_Motors(900);
    for (volatile int i = 0; i < 6000000; i++) __NOP();
}

void Motors_Stop(void) {
    Set_Motors(900);
}

void Motors_Idle(void) {
    Set_Motors(1050);
}

void Motors_Arm(void) {
    Set_Motors(1050);
}

void Motors_Disarm(void) {
    Set_Motors(900);
}

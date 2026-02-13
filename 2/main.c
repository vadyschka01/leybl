#include "stm32f4xx.h"

// Функция задержки
void delay(int count) {
    while(count--) __NOP();
}

int main() {
    // 1. Включаем тактирование порта D и Таймера 4
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; 
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // 2. Настраиваем ножку PD12 (Зеленый диод)
    // Режим: Alternate Function (AF) = 10
    GPIOD->MODER &= ~(3UL << (12 * 2)); 
    GPIOD->MODER |=  (2UL << (12 * 2)); 

    // Выбираем AF2 (TIM4) для PD12. Это регистр AFR[1] (High)
    GPIOD->AFR[1] &= ~(0xF << 16); // Чистим биты
    GPIOD->AFR[1] |=  (0x2 << 16); // Ставим "2" (AF2)

    // 3. Настройка Таймера 4
    // PSC: Делим частоту процессора (16 МГц) на 16 -> получаем 1 МГц
    TIM4->PSC = 16 - 1; 
    // ARR: Считаем до 1000 (Период ШИМ)
    TIM4->ARR = 1000 - 1;

    // 4. Настройка канала 1 (ШИМ режим)
    TIM4->CCMR1 &= ~TIM_CCMR1_OC1M; 
    TIM4->CCMR1 |= (6UL << 4);      // PWM Mode 1
    TIM4->CCMR1 |= TIM_CCMR1_OC1PE; // Preload enable
    TIM4->CCER |= TIM_CCER_CC1E;    // Включаем выход

    // 5. Запуск таймера
    TIM4->CR1 |= TIM_CR1_CEN;

    int brightness = 0;
    int step = 10;

    while (1) {
        // Меняем яркость (ширину импульса)
        TIM4->CCR1 = brightness;

        brightness += step;

        // Туда-сюда
        if (brightness >= 1000 || brightness <= 0) {
            step = -step;
        }

        delay(5000); // Скорость изменения яркости
    }
    return 0;
}
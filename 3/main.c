#include "stm32f4xx.h"
#include <stdio.h>

// Функция простой задержки
void delay(int count) {
    while(count--) {
        __NOP();
    }
}

int main() {
    // --- 1. Инициализация GPIO (Светодиод) ---
    // Включаем тактирование порта I
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
    // Настраиваем ножку PI11 на выход (General Purpose Output)
    GPIOI->MODER &= ~(3UL << (11 * 2)); // Сброс бит
    GPIOI->MODER |= (1UL << (11 * 2));  // Установка режима

    // --- 2. Инициализация ADC (Потенциометр) ---
    // Включаем тактирование порта C (для POT1 на PC3)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    // Настраиваем PC3 как аналоговый вход
    GPIOC->MODER |= (3UL << (3 * 2));
    
    // Включаем тактирование модуля АЦП1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    // Включаем сам АЦП
    ADC1->CR2 |= ADC_CR2_ADON;
    // Выбираем 13-й канал (PC3) для регулярной группы преобразований
    ADC1->SQR3 = 13;

    // --- 3. Основной цикл ---
    while (1) {
        // Запуск преобразования АЦП
        ADC1->CR2 |= ADC_CR2_SWSTART;
        
        // Ожидание окончания преобразования (флаг EOC)
        while(!(ADC1->SR & ADC_SR_EOC));

        // Чтение результата из регистра данных
        int adc_value = ADC1->DR;

        // Вывод данных в терминал отладчика
        printf("Potentiometer: %d\n", adc_value);
        
        // Индикация работы (переключение светодиода)
        GPIOI->ODR ^= (1UL << 11);

        // Задержка для читаемости данных
        delay(500000); 
    }
    
    return 0;
}

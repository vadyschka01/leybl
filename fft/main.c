#include "stm32g4xx.h"
#include "imu.h"

// Флаг для синхронизации цикла
volatile uint8_t imu_flag = 0;

// Структура бинарного пакета для передачи на ПК (14 байт)
#pragma pack(push, 1)
typedef struct {
    uint8_t  header[2]; // 0xAA, 0xBB
    int16_t  gx;        // Гироскоп X
    int16_t  gy;        // Гироскоп Y
    int16_t  gz;        // Гироскоп Z
    int16_t  ax;        // Акселерометр X
    int16_t  ay;        // Акселерометр Y
    int16_t  az;        // Акселерометр Z
} LogPacket_t;
#pragma pack(pop)

// Функция быстрой отправки пакета в UART
void UART_SendPacket(LogPacket_t *p) {
    uint8_t *ptr = (uint8_t*)p;
    for (uint16_t i = 0; i < sizeof(LogPacket_t); i++) {
        // Ждем, пока регистр данных LPUART освободится
        while (!(LPUART1->ISR & USART_ISR_TXE));
        LPUART1->TDR = ptr[i];
    }
}

// Настройка LPUART1 на скорость 921600 (для 1000 Гц это важно)
void LPUART1_Log_Init(void)
{
    // 1. Включаем тактирование GPIOA и LPUART1
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;

    // 2. Настройка ПИНОВ: PA2 (TX) и PA3 (RX)
    // В SBUS у тебя был только PA3, тут добавляем PA2
    GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
    GPIOA->MODER |=  (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1); // AF mode
    
    // AF12 для обоих пинов
    GPIOA->AFR[0] &= ~((0xF << 8) | (0xF << 12));
    GPIOA->AFR[0] |=  ((12 << 8) | (12 << 12));

    // 3. Выбор источника тактирования (как в твоем sbus.c)
    // 2 (0x10) - это HSI16 (16 МГц)
    RCC->CCIPR &= ~RCC_CCIPR_LPUART1SEL;
    RCC->CCIPR |= (2 << RCC_CCIPR_LPUART1SEL_Pos); 

    // 4. Настройка параметров LPUART
    LPUART1->CR1 = 0;
    LPUART1->CR2 = 0; // ВАЖНО: Убираем инверсию и ставим 1 стоп-бит
    LPUART1->CR3 = 0;

    // Расчет скорости для HSI (16 МГц)
    // Формула: BRR = (256 * 16,000,000) / 921,600 = 4444 (округлим)
    LPUART1->BRR = 4444; 

    // Включаем TE (передатчик), RE (приемник) и UE (модуль)
    // БЕЗ чётности (PCE), чтобы компьютер понимал данные
    LPUART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

// TIM6 на 1000 Гц
void TIM6_Init_1000Hz(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    TIM6->PSC = 16000 - 1;   // 160 MHz / 16000 = 10 000 тиков/сек
    TIM6->ARR = 10 - 1;      // 10 000 / 10 = 1000 Гц
    TIM6->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 |= TIM_CR1_CEN;
}

void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR = 0;
        imu_flag = 1;
    }
}

int main(void) {
    // Настройка тактирования (HSI 160MHz)
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
    
    // Инициализация периферии
    I2C1_Init();
    LPUART1_Log_Init();
    IMU_Init();
    
    // Калибровка гироскопа (дрон должен лежать неподвижно!)
    IMU_Calibrate(); 
    
    TIM6_Init_1000Hz();

    LogPacket_t packet;
    packet.header[0] = 0xAA;
    packet.header[1] = 0xBB;

    while (1) {
        if (imu_flag) {
            imu_flag = 0;

            // 1. Читаем данные и пропускаем через текущие фильтры
            IMU_ReadRaw(); 

            // 2. Формируем пакет для отправки
      // Мы берем значения из imu.c (там они называются raw_gx, raw_gy и т.д.)
      // И кладем их в нашу структуру пакета
      packet.gx = raw_gx;
      packet.gy = raw_gy;
      packet.gz = raw_gz;
      packet.ax = raw_ax;
      packet.ay = raw_ay;
      packet.az = raw_az;

      // И отправляем
      UART_SendPacket(&packet);

            // 3. Плюем данные в UART
            UART_SendPacket(&packet);
        }
    }
}
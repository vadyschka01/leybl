#include "stm32g4xx.h"
#include "imu.h"
#include "motors.h" // Если подключен, но мы отключили моторы для логов

// Флаг синхронизации таймера 1000 Гц
volatile uint8_t imu_flag = 0;

// --- СТРУКТУРА ПАКЕТА (НОВАЯ ДЛЯ 3 ОСТЕЙ) ---
#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];      // [0:1] — Заголовок AA BB
    int16_t gx_raw;         // [2:3]  — Гироскоп X (сырой)
    int16_t gy_raw;         // [4:5]  — Гироскоп Y (сырой)
    int16_t gz_raw;         // [6:7]  — Гироскоп Z (сырой)
    int16_t ax;             // [8:9]  — Акселерометр X
    int16_t ay;             // [10:11] — Акселерометр Y
    int16_t az;             // [12:13] — Акселерометр Z
    int16_t filt_gx;        // [14:15] — Гироскоп X (отфильтрованный)
    int16_t filt_gy;        // [16:17] — Гироскоп Y (отфильтрованный)
    int16_t filt_gz;        // [18:19] — Гироскоп Z (отфильтрованный)
} LogPacket_t;              // ИТОГО: 20 БАЙТ
#pragma pack(pop)

// --- НАСТРОЙКА ВЕКТОРА УПРАВЛЕНИЯ ПРЕРЫВАНИЯМИ TIM6 ---
// Для частоты 160 МГц / 16000 = 10 000 Гц, ARR = 9 => 1000 Гц
void TIM6_Init_1000Hz(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    TIM6->PSC = 16000 - 1;
    TIM6->ARR = 10 - 1; // Важно: 1 мс период
    TIM6->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM6_DAC_IRQn, 1);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 |= TIM_CR1_CEN;
}

// --- ОБРАБОТЧИК ПРЕРЫВАНИЯ ---
void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR = 0;
        imu_flag = 1; // Разрешаем считывать датчик раз в 1 мс
    }
}

// --- СКОРОСТНОЙ UART (PB3/TX, PB4/RX) ---
// Под 160 МГц и 921600 бод: BRR = 174
void UART2_Log_Init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    
    // Выбираем SYSCLK как источник (для точности скоростей)
    RCC->CCIPR &= ~RCC_CCIPR_USART2SEL;
    RCC->CCIPR |= (1 << RCC_CCIPR_USART2SEL_Pos);

    // Настройка пинов PB3(TX) и PB4(RX) на AF7 (USART2)
    GPIOB->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4);
    GPIOB->MODER |= (GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_1);
    GPIOB->AFR[0] &= ~((0xF << 12) | (0xF << 16));
    GPIOB->AFR[0] |= ((7 << 12) | (7 << 16));

    USART2->BRR = 174; // Калькулятор: 160MHz / 921600 ≈ 173.6 -> 174
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void UART_SendPacket(LogPacket_t *p) {
    uint8_t *ptr = (uint8_t *)p;
    for (uint16_t i = 0; i < sizeof(LogPacket_t); i++) {
        while (!(USART2->ISR & USART_ISR_TXE));
        USART2->TDR = ptr[i];
    }
}

// --- ОСНОВНОЙ ЦИКЛ ---
int main(void) {
    // Инициализация тактирования и периферии
    SystemClock_Config_My(); // Твой код настройки 160МГц
    I2C1_Init();            // Из vr_imu.txt
    UART2_Log_Init();       // Наш код выше
    
    // Калибровка гироскопов (обязательно! иначе будут смещения)
    IMU_Calibrate();
    IMU_InitFilters();      // Новый вызов для настройки фильтров

    TIM6_Init_1000Hz();
    __enable_irq();

    LogPacket_t packet;
    packet.header[0] = 0xAA;
    packet.header[1] = 0xBB;

    while (1) {
        if (imu_flag) {
            imu_flag = 0;
            
            // Чтение сырых данных
            IMU_ReadRawData(); 
            
            // Формирование пакета
            packet.gx_raw = raw_gx;
            packet.gy_raw = raw_gy;
            packet.gz_raw = raw_gz;
            packet.ax = raw_ax;
            packet.ay = raw_ay;
            packet.az = raw_az;
            packet.filt_gx = (int16_t)filt_gx;
            packet.filt_gy = (int16_t)filt_gy;
            packet.filt_gz = (int16_t)filt_gz;
            
            // Отправка данных
            UART_SendPacket(&packet);
        }
    }
}
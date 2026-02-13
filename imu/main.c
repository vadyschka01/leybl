#include "stm32g4xx.h"
#include <stdint.h>

// ================== SBUS ГЛОБАЛЬНЫЕ ==================

// Каналы SBUS (0–2047)
volatile uint16_t rc_channels[16];
volatile uint8_t sbus_failsafe = 0;
volatile uint8_t sbus_frame_lost = 0;

// Счётчики
volatile uint32_t sbus_packets_ok = 0;
volatile uint32_t sbus_decode_errors = 0;

// Буфер пакета
volatile uint8_t rx_buffer[25];
volatile uint8_t rx_idx = 0;
volatile uint8_t parser_state = 0;

// Таймеры
volatile uint32_t ms = 0;                  // миллисекунды (SysTick)
volatile uint32_t sbus_last_byte_time = 0; // время последнего байта
volatile uint32_t sbus_hz = 0;             // частота SBUS
volatile uint32_t sbus_hz_counter = 0;     // пакеты за секунду

// ================== IMU ГЛОБАЛЬНЫЕ ==================

#define IMU_ADDR      (0x68 << 1)  // как в imu_pwm.txt
#define REG_WHO_AM_I  0x00         // для ICM-20948 в Bank 0
#define REG_ACCEL_GYRO_START 0x2D  // начало блока аксель+гиро в Bank 0

volatile uint8_t  imu_whoami = 0;
volatile int16_t  imu_ax = 0;
volatile int16_t  imu_ay = 0;
volatile int16_t  imu_az = 0;
volatile int16_t  imu_gx = 0;
volatile int16_t  imu_gy = 0;
volatile int16_t  imu_gz = 0;

// ================== ЗАДЕРЖКА ==================
static void delay_long(int limit) {
    for (volatile int i = 0; i < limit; i++) {
        __NOP();
    }
}

// ================== SysTick ==================
void SysTick_Handler(void)
{
    ms++;
}

// ================== SBUS ДЕКОДЕР ==================
void SBUS_Decode(void)
{
    if (rx_buffer[0] != 0x0F || rx_buffer[24] != 0x00) {
        sbus_decode_errors++;
        return;
    }

    rc_channels[0]  = ((rx_buffer[1]    | rx_buffer[2] << 8)                 & 0x07FF);
    rc_channels[1]  = ((rx_buffer[2]>>3 | rx_buffer[3] << 5)                 & 0x07FF);
    rc_channels[2]  = ((rx_buffer[3]>>6 | rx_buffer[4] << 2 | rx_buffer[5] << 10) & 0x07FF);
    rc_channels[3]  = ((rx_buffer[5]>>1 | rx_buffer[6] << 7)                 & 0x07FF);
    rc_channels[4]  = ((rx_buffer[6]>>4 | rx_buffer[7] << 4)                 & 0x07FF);
    rc_channels[5]  = ((rx_buffer[7]>>7 | rx_buffer[8] << 1 | rx_buffer[9] << 9)  & 0x07FF);
    rc_channels[6]  = ((rx_buffer[9]>>2 | rx_buffer[10] << 6)                & 0x07FF);
    rc_channels[7]  = ((rx_buffer[10]>>5| rx_buffer[11] << 3)                & 0x07FF);
    rc_channels[8]  = ((rx_buffer[12]   | rx_buffer[13] << 8)                & 0x07FF);
    rc_channels[9]  = ((rx_buffer[13]>>3| rx_buffer[14] << 5)                & 0x07FF);
    rc_channels[10] = ((rx_buffer[14]>>6| rx_buffer[15] << 2 | rx_buffer[16] << 10) & 0x07FF);
    rc_channels[11] = ((rx_buffer[16]>>1| rx_buffer[17] << 7)                & 0x07FF);
    rc_channels[12] = ((rx_buffer[17]>>4| rx_buffer[18] << 4)                & 0x07FF);
    rc_channels[13] = ((rx_buffer[18]>>7| rx_buffer[19] << 1 | rx_buffer[20] << 9)  & 0x07FF);
    rc_channels[14] = ((rx_buffer[20]>>2| rx_buffer[21] << 6)                & 0x07FF);
    rc_channels[15] = ((rx_buffer[21]>>5| rx_buffer[22] << 3)                & 0x07FF);

    sbus_frame_lost = (rx_buffer[23] & 0x04) ? 1 : 0;
    sbus_failsafe   = (rx_buffer[23] & 0x08) ? 1 : 0;

    sbus_packets_ok++;
    sbus_hz_counter++;   // для измерения частоты
}

// ================== LPUART1 ДЛЯ SBUS ==================
void LPUART1_SBUS_Init(void)
{
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;

    // PA3 = AF12 (как в рабочем варианте)
    GPIOA->MODER &= ~(GPIO_MODER_MODE3);
    GPIOA->MODER |=  GPIO_MODER_MODE3_1;
    GPIOA->AFR[0] &= ~(0xF << 12);
    GPIOA->AFR[0] |=  (12 << 12);

    LPUART1->CR1 = 0;
    LPUART1->CR2 = 0;
    LPUART1->CR3 = 0;

    // Источник тактирования LPUART1 = HSI16
    RCC->CCIPR &= ~RCC_CCIPR_LPUART1SEL;
    RCC->CCIPR |= (2 << RCC_CCIPR_LPUART1SEL_Pos);

    // Скорость 100000 (как в твоём коде)
    LPUART1->BRR = SystemCoreClock / (100000 / 256);

    // Parity ODD
    LPUART1->CR1 |= USART_CR1_PCE | USART_CR1_PS;

    // Инверсия + 2 стоп-бита
    LPUART1->CR2 |= USART_CR2_RXINV;
    LPUART1->CR2 |= USART_CR2_STOP_1;

    // Прерывание RX
    LPUART1->CR1 |= USART_CR1_RXNEIE;

    // Включаем UART
    LPUART1->CR1 |= USART_CR1_RE | USART_CR1_UE;

    NVIC_SetPriority(LPUART1_IRQn, 1);
    NVIC_EnableIRQ(LPUART1_IRQn);
}

// ================== IRQ LPUART1 ==================
void LPUART1_IRQHandler(void)
{
    if (LPUART1->ISR & USART_ISR_RXNE)
    {
        uint8_t data = (uint8_t)LPUART1->RDR;
        sbus_last_byte_time = ms;   // отметка времени

        if (parser_state == 0) {
            if (data == 0x0F) {
                rx_buffer[0] = data;
                rx_idx = 1;
                parser_state = 1;
            }
        } else {
            rx_buffer[rx_idx++] = data;

            if (rx_idx >= 25) {
                SBUS_Decode();
                parser_state = 0;
                rx_idx = 0;
            }
        }
    }

    if (LPUART1->ISR & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) {
        LPUART1->ICR = USART_ICR_ORECF | USART_ICR_NECF |
                       USART_ICR_FECF | USART_ICR_PECF;
    }
}

// ================== I2C1 (как в imu_pwm.txt, + burst) ==================
void I2C1_Init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

    // PB8 (SCL), PB9 (SDA) -> AF4, Open Drain
    GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOB->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);
    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0); // Pull-up
    GPIOB->AFR[1] &= ~((0xF << 0) | (0xF << 4));
    GPIOB->AFR[1] |= ((4 << 0) | (4 << 4)); // AF4

    // Тайминги для 100 кГц
    I2C1->TIMINGR = 0x00303D5B;
    I2C1->CR1 |= I2C_CR1_PE;
}

uint8_t I2C_ReadReg(uint8_t devAddr, uint8_t regAddr) {
    // Запись адреса регистра
    I2C1->CR2 = (devAddr & I2C_CR2_SADD) | (1 << 16) | I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXIS)) {
        if (I2C1->ISR & I2C_ISR_NACKF) return 0xFF;
    }
    I2C1->TXDR = regAddr;
    while (!(I2C1->ISR & I2C_ISR_TC));

    // Чтение данных
    I2C1->CR2 = (devAddr & I2C_CR2_SADD) | (1 << 16) |
                I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;
    while (!(I2C1->ISR & I2C_ISR_RXNE));
    return (uint8_t)I2C1->RXDR;
}

void I2C_ReadMulti(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint8_t len) {
    // 1) Записываем адрес регистра
    I2C1->CR2 = (devAddr & I2C_CR2_SADD) | (1 << 16) | I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXIS)) {
        if (I2C1->ISR & I2C_ISR_NACKF) return;
    }
    I2C1->TXDR = regAddr;
    while (!(I2C1->ISR & I2C_ISR_TC));

    // 2) Читаем len байт
    I2C1->CR2 = (devAddr & I2C_CR2_SADD) | ((uint32_t)len << 16) |
                I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;

    for (uint8_t i = 0; i < len; i++) {
        while (!(I2C1->ISR & I2C_ISR_RXNE));
        buf[i] = (uint8_t)I2C1->RXDR;
    }
}

// ================== IMU INIT + READ ==================
static void IMU_WriteReg(uint8_t reg, uint8_t val) {
    // простая запись: devAddr + reg + val
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

static void IMU_SetBank(uint8_t bank) {
    // REG_BANK_SEL = 0x7F
    IMU_WriteReg(0x7F, (uint8_t)(bank << 4));
}

void IMU_Init(void) {
    // Сброс и включение, как в imu.cpp, но упрощённо
    IMU_SetBank(0);
    IMU_WriteReg(0x06, 0x80); // PWR_MGMT_1 reset
    delay_long(100000);
    IMU_WriteReg(0x06, 0x01); // clock auto
    delay_long(100000);

    // Можно добавить базовую настройку фильтров/диапазонов при желании
}

void IMU_ReadAccelGyro(void) {
    uint8_t buf[14];

    IMU_SetBank(0);
    I2C_ReadMulti(IMU_ADDR, REG_ACCEL_GYRO_START, buf, 14);

    // Данные big-endian: hi, lo
    imu_ax = (int16_t)((buf[0] << 8) | buf[1]);
    imu_ay = (int16_t)((buf[2] << 8) | buf[3]);
    imu_az = (int16_t)((buf[4] << 8) | buf[5]);
    imu_gx = (int16_t)((buf[6] << 8) | buf[7]);
    imu_gy = (int16_t)((buf[8] << 8) | buf[9]);
    imu_gz = (int16_t)((buf[10] << 8) | buf[11]);
    // buf[12..13] — temp, если нужно, можно тоже сохранить
}

// ================== MAIN ==================
int main(void)
{
    // Включаем HSI 16 MHz
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // SysTick = 1 мс
    SysTick_Config(SystemCoreClock / 1000U);

    // Инициализация SBUS и I2C
    LPUART1_SBUS_Init();
    I2C1_Init();
    IMU_Init();

    uint32_t last_hz_ms  = 0;
    uint32_t last_imu_ms = 0;
    uint32_t last_id_ms  = 0;

    while (1)
    {
        // === Таймаут парсера SBUS ===
        if (ms - sbus_last_byte_time > 30U) {
            parser_state = 0;
            rx_idx = 0;
        }

        // === Частота SBUS ===
        if (ms - last_hz_ms >= 1000U) {
            sbus_hz = sbus_hz_counter;
            sbus_hz_counter = 0;
            last_hz_ms = ms;
        }

        // === Периодическое чтение WHO_AM_I ===
        if (ms - last_id_ms >= 500U) { // раз в 500 мс
            IMU_SetBank(0);
            imu_whoami = I2C_ReadReg(IMU_ADDR, REG_WHO_AM_I);
            last_id_ms = ms;
        }

        // === Периодическое чтение аксель/гиро ===
        if (ms - last_imu_ms >= 20U) { // 50 Гц
            IMU_ReadAccelGyro();
            last_imu_ms = ms;
        }

        __NOP(); // чтобы Live Watch не душил оптимизацию
    }
}

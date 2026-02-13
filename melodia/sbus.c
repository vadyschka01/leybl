#include "stm32g4xx.h"
#include "sbus.h"

// === Глобальные переменные ===
volatile uint16_t rc_channels[16];
volatile uint8_t sbus_failsafe = 0;
volatile uint8_t sbus_frame_lost = 0;

volatile uint32_t sbus_packets_ok = 0;
volatile uint32_t sbus_decode_errors = 0;

volatile uint8_t  rx_buffer[25];
volatile uint8_t  rx_idx = 0;
volatile uint8_t  parser_state = 0;

volatile uint32_t sbus_last_byte_time = 0;
volatile uint32_t sbus_hz = 0;
volatile uint32_t sbus_hz_counter = 0;

// === SBUS ДЕКОДЕР ===
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
    sbus_hz_counter++;
}

// === ИНИЦИАЛИЗАЦИЯ LPUART1 ===
void LPUART1_SBUS_Init(void)
{
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;

    GPIOA->MODER &= ~(GPIO_MODER_MODE3);
    GPIOA->MODER |=  GPIO_MODER_MODE3_1;
    GPIOA->AFR[0] &= ~(0xF << 12);
    GPIOA->AFR[0] |=  (12 << 12);

    LPUART1->CR1 = 0;
    LPUART1->CR2 = 0;
    LPUART1->CR3 = 0;

    RCC->CCIPR &= ~RCC_CCIPR_LPUART1SEL;
    RCC->CCIPR |= (2 << RCC_CCIPR_LPUART1SEL_Pos);

    LPUART1->BRR = SystemCoreClock / (100000 / 256);

    LPUART1->CR1 |= USART_CR1_PCE | USART_CR1_PS;
    LPUART1->CR2 |= USART_CR2_RXINV | USART_CR2_STOP_1;

    LPUART1->CR1 |= USART_CR1_RXNEIE;
    LPUART1->CR1 |= USART_CR1_RE | USART_CR1_UE;

    NVIC_SetPriority(LPUART1_IRQn, 1);
    NVIC_EnableIRQ(LPUART1_IRQn);
}

// === IRQ ===
void LPUART1_IRQHandler(void)
{
    if (LPUART1->ISR & USART_ISR_RXNE)
    {
        uint8_t data = (uint8_t)LPUART1->RDR;
        sbus_last_byte_time = *((volatile uint32_t*)0xE000E018); // SysTick->VAL alternative

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

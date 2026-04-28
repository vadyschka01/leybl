#include "stm32g4xx.h"
#include "imu.h"
#include "motors.h"
#include "dsp_manager.h"

// 1. ПЕРЕМЕННЫЕ
volatile uint8_t imu_flag = 0;

volatile uint32_t m1_speed = 900;
volatile uint32_t m2_speed = 900;
volatile uint32_t m3_speed = 900;
volatile uint32_t m4_speed = 900;

#pragma pack(push, 1)
typedef struct {
    uint8_t  header[2];
    int16_t  gx;
    int16_t  filt_gx;
} Telemetry_t;
#pragma pack(pop)

// 2. ПРОТОТИПЫ (Чтобы компилятор не ругался)
void SystemClock_Config_160MHz(void);
void UART2_Init_921600(void);
void TIM6_Init_1000Hz(void);
void UART_SendPacket(Telemetry_t *p);

// 3. ОСНОВНОЙ ЦИКЛ
int main(void) {
    SystemClock_Config_160MHz();
    Motors_Init();
    Set_Motors(900);
    for (int i = 0; i < 20000000; i++) __NOP(); // Пауза для ESC

    I2C1_Init();
    UART2_Init_921600();
    IMU_Init();
    IMU_Calibrate();
    
    DSP_Init(); // Инициализация нашего анализатора Фурье
    
    TIM6_Init_1000Hz();
    __enable_irq();

    Telemetry_t pkt;
    pkt.header[0] = 0xAA; pkt.header[1] = 0xBB;

    while (1) {
        if (imu_flag) {
              imu_flag = 0;

              IMU_ReadRawData(); 

              //сохраняем во временную переменную
              int16_t gx_val = raw_gx; 
              DSP_AddSample((float32_t)gx_val);

              if (dsp_buffer_ready) {
                  DSP_Process(); 
              }

              pkt.gx = gx_val;
              pkt.filt_gx = (int16_t)filt_gx;
              UART_SendPacket(&pkt);

              Set_Motor_Individual(m1_speed, m2_speed, m3_speed, m4_speed);
          }
        }
    }

// 4. РЕАЛИЗАЦИЯ ФУНКЦИЙ (Тут был провал - их не хватало!)

void SystemClock_Config_160MHz(void) {
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));
    RCC->PLLCFGR = (RCC_PLLCFGR_PLLSRC_HSI | (3 << RCC_PLLCFGR_PLLM_Pos) | 
                   (80 << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLR_Pos));
    RCC->CR |= RCC_CR_PLLON;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;
    while(!(RCC->CR & RCC_CR_PLLRDY));
    FLASH->ACR |= (4 << FLASH_ACR_LATENCY_Pos);
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    SystemCoreClockUpdate();
}

void UART2_Init_921600(void) {
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    RCC->CCIPR    |= (1 << RCC_CCIPR_USART2SEL_Pos);
    GPIOB->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4);
    GPIOB->MODER |=  (GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_1);
    GPIOB->AFR[0] &= ~((0xF << 12) | (0xF << 16));
    GPIOB->AFR[0] |=  ((7 << 12) | (7 << 16));
    USART2->BRR = 174; 
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void UART_SendPacket(Telemetry_t *p) {
    uint8_t *ptr = (uint8_t*)p;
    for (uint16_t i = 0; i < sizeof(Telemetry_t); i++) {
        while (!(USART2->ISR & USART_ISR_TXE));
        USART2->TDR = ptr[i];
    }
}

void TIM6_Init_1000Hz(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    TIM6->PSC = 16000 - 1; 
    TIM6->ARR = 10 - 1;
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
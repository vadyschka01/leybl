#include "stm32g4xx.h"

#include "gpio.h"

#include "dshot.h"

#define DSHOT_FREQ_SCALE 10'000'000

#define DSHOT_FRAME_DATA 16 
#define DSHOT_FRAME_SIZE 20

#define DSHOT_TIM   4

#define DSHOT_TIM2  0
#define DSHOT_TIM1  1
#define DSHOT_TIM16 2
#define DSHOT_TIM17 3

#define DSHOT_CCR  2
#define DSHOT_CCR1 0
#define DSHOT_CCR2 1
#define DSHOT_CCR3 0
#define DSHOT_CCR4 1

#define DSHOT_BUF 3

static bool DSHOT_Bidirect;

static unsigned short DSHOT_Comm[DSHOT_TIM][DSHOT_CCR] = { {1, 1}, {1, 1} };

static bool DSHOT_Wait[DSHOT_TIM] = { false, false };
static unsigned short DSHOT_Shift, DSHOT_Timer, DSHOT_Waiting, DSHOT_PeriodTX, DSHOT_PeriodRX, DSHOT_Bit_0, DSHOT_Bit_1;

static unsigned char DSHOT_tx_li[DSHOT_TIM] = { 0x11, 0x11 }; // buf lock: 1, 2, 3 // buf id: 1, 2, 3 
static unsigned short DSHOT_tx_buf[DSHOT_TIM][DSHOT_BUF][DSHOT_FRAME_SIZE*DSHOT_CCR]; // TIM | triple buffer | CCR1 + CCR2

static unsigned char DSHOT_rx_li[DSHOT_TIM] = { 0x11, 0x11 }; // buf lock: 1, 2, 3 // buf id: 1, 2, 3 
static unsigned char DSHOT_rx_sel[DSHOT_TIM]; // Selected buffer
static unsigned short DSHOT_rx_buf[DSHOT_TIM][DSHOT_BUF][DSHOT_CCR][DSHOT_FRAME_SIZE]; // TIM | triple buffer | CCRx | signal
static unsigned char DSHOT_rx_cnt[DSHOT_TIM][DSHOT_BUF][DSHOT_CCR]; // TIM | triple buffer | data count

static unsigned short DSHOT_ERPM[DSHOT_TIM][DSHOT_CCR] = { {0, 0}, {0, 0} };

// Таблица декодирования GCR (DShot RPM)
static const unsigned char GCR_DECODE_TABLE[32]
{
  [0x19] = 0x0, // 11001b -> 0
  [0x1B] = 0x1, // 11011b -> 1
  [0x12] = 0x2, // 10010b -> 2
  [0x13] = 0x3, // 10011b -> 3
  [0x1D] = 0x4, // 11101b -> 4
  [0x15] = 0x5, // 10101b -> 5
  [0x16] = 0x6, // 10110b -> 6
  [0x17] = 0x7, // 10111b -> 7
  [0x1A] = 0x8, // 11010b -> 8
  [0x09] = 0x9, // 01001b -> 9
  [0x0A] = 0xA, // 01010b -> A
  [0x0B] = 0xB, // 01011b -> B
  [0x1E] = 0xC, // 11110b -> C
  [0x0D] = 0xD, // 01101b -> D
  [0x0E] = 0xE, // 01110b -> E
  [0x0F] = 0xF, // 01111b -> F
  
  // Остальные значения 0xFF (ошибки)
  [0x00] = 0xFF, [0x01] = 0xFF, [0x02] = 0xFF, [0x03] = 0xFF,
  [0x04] = 0xFF, [0x05] = 0xFF, [0x06] = 0xFF, [0x07] = 0xFF,
  [0x08] = 0xFF, [0x0C] = 0xFF, [0x10] = 0xFF, [0x11] = 0xFF,
  [0x14] = 0xFF, [0x18] = 0xFF, [0x1C] = 0xFF, [0x1F] = 0xFF
};

inline unsigned char decodeGCR(unsigned char gcr_val)
{
  return GCR_DECODE_TABLE[gcr_val & 0x1F]; // gcr_val должен быть от 0 до 31 (5 бит)
}
//------------------------------------------------------------------------------

static unsigned long CalcERPM(unsigned short* buff, unsigned long size, unsigned long count)
{
  if(count>DSHOT_FRAME_SIZE) return -1;
  
  unsigned int fb=0;
  bool bit=false;
  int cnt = 0;
  
  for (int a = 0; a < count; a++)
  {
    buff[a] = buff[a+1] - buff[a];

    int t=buff[a];
    int n, x;
    if (t > DSHOT_Bit_0*5)  { n = 3; x = 7; }
    else if (t > DSHOT_Bit_0*3) { n = 2; x = 3; }
    else if (t > DSHOT_Bit_0) { n = 1; x = 1; }
    else { n = 0; x = 0; }
    
    fb<<=n;
    if(bit) fb |= x;
    
    bit=!bit;
    cnt+=n;
  }
  
  int last = 21-cnt;
  fb<<=last;
  fb|=(1U << last) - 1;
  
  fb = fb ^ (fb>>1);
  
  const unsigned long mask = 0x1F; 

  unsigned int gcr1 = (fb >> 0)  & mask;
  unsigned int gcr2 = (fb >> 5)  & mask;
  unsigned int gcr3 = (fb >> 10) & mask;
  unsigned int gcr4 = (fb >> 15) & mask;
  
  gcr1=decodeGCR(gcr1);
  gcr2=decodeGCR(gcr2);
  gcr3=decodeGCR(gcr3);
  gcr4=decodeGCR(gcr4);
  
  int crc = (~(gcr2 ^ gcr3 ^ gcr4)) & 0x0F;

  bool valid = crc==gcr1;
  
  if (!valid || gcr1==0xFF || gcr2==0xFF || gcr3==0xFF || gcr4==0xFF) return -1;
  
  unsigned short data = (gcr4 << 8) | (gcr3 << 4) | gcr2;
  
  int e = (data >> 9) & 0x07; // Старшие 3 бита
  int m = data & 0x1FF; 
  
  int p = m<<e;
  
  if (p==0) return -1;
  
  return 1'000'000/p;
}
//------------------------------------------------------------------------------

static unsigned char buf_lock(unsigned char* buf_li) // Lock for read
{
  unsigned char li, i;
  
  do // Try lock
  {
    li = __LDREXB(buf_li); // Read lock+id
    i = li & 0x0F; // Get id
    li = i | i<<4; // Set lock = id
  } while (__STREXB(li, buf_li));
  
  return i-1; // id [1,2,3] -> index [0,1,2]
}
//------------------------------------------------------------------------------

static unsigned char buf_select(unsigned char* buf_li) // Prepare to write
{
  unsigned char li=*buf_li, l, i, n;

  l = li >> 4; // Get lock
  i = li & 0x0F; // Get id
  n = i;
  
  do
  {
    n++;
    if(n>DSHOT_BUF) n=1;
  } while (n==l || n==i);
  
  return n-1; // Next index (free to write)
}
//------------------------------------------------------------------------------

static void buf_change(unsigned char* buf_li, unsigned char next) // Relese written
{
  unsigned char li;
  
  do // Try change
  {
    li = __LDREXB(buf_li); // Read lock+id
    li = (li & 0xF0) | (next+1); // Prev lock + new id
  } while (__STREXB(li, buf_li));
}
//------------------------------------------------------------------------------

static bool buf_new(unsigned char* buf_li) // New data
{
  unsigned char li = *buf_li;
  
  return (li >> 4) != (li & 0x0F);
}
//------------------------------------------------------------------------------

static void ModeTIM2_TX();
static void ModeTIM1_TX();
static void ModeTIM16_TX();
static void ModeTIM17_TX();

static void ModeTIM2_RX();
static void ModeTIM1_RX();

static void ModeTIM2_WAIT();
static void ModeTIM1_WAIT();
static void ModeTIM16_WAIT();
static void ModeTIM17_WAIT();

extern "C" void DMA2_Channel1_IRQHandler()
{
  unsigned long sr = DMA2->ISR;
  if(sr & DMA_ISR_TCIF1)
  {
    DMA2->IFCR = DMA_IFCR_CTCIF1;
    
    if(DSHOT_Bidirect) ModeTIM2_RX();
    else ModeTIM2_WAIT();
  }
}
//------------------------------------------------------------------------------

extern "C" void DMA2_Channel2_IRQHandler()
{
  unsigned long sr = DMA2->ISR;
  if(sr & DMA_ISR_TCIF2)
  {
    DMA2->IFCR = DMA_IFCR_CTCIF2;
    
    if(DSHOT_Bidirect) ModeTIM1_RX();
    else ModeTIM1_WAIT();
  }
}
//------------------------------------------------------------------------------

extern "C" void DMA1_Channel5_IRQHandler()
{
  unsigned long sr = DMA1->ISR;
  if(sr & DMA_ISR_TCIF5)
  {
    DMA1->IFCR = DMA_IFCR_CTCIF5;
    
    ModeTIM16_WAIT();
  }
}
//------------------------------------------------------------------------------

extern "C" void DMA1_Channel6_IRQHandler()
{
  unsigned long sr = DMA1->ISR;
  if(sr & DMA_ISR_TCIF6)
  {
    DMA1->IFCR = DMA_IFCR_CTCIF6;
    
    ModeTIM17_WAIT();
  }
}
//------------------------------------------------------------------------------

extern "C" void TIM2_IRQHandler()
{
  if(DSHOT_Wait[DSHOT_TIM2])
  {
    ModeTIM2_TX();
  }
  else
  {
    unsigned char index=DSHOT_rx_sel[DSHOT_TIM2];
    
    DSHOT_rx_cnt[DSHOT_TIM2][index][DSHOT_CCR1]=DSHOT_FRAME_SIZE-DMA2_Channel3->CNDTR-1;
    DSHOT_rx_cnt[DSHOT_TIM2][index][DSHOT_CCR2]=DSHOT_FRAME_SIZE-DMA2_Channel4->CNDTR-1;
    
    ModeTIM2_WAIT();
  }
}
//------------------------------------------------------------------------------

extern "C" void TIM1_UP_TIM16_IRQHandler()
{
  if((TIM1->SR & TIM1->DIER) & TIM_SR_UIF)
  {
    if(DSHOT_Wait[DSHOT_TIM1])
    {
      ModeTIM1_TX();
    }
    else
    {
      unsigned char index=DSHOT_rx_sel[DSHOT_TIM1];
      
      DSHOT_rx_cnt[DSHOT_TIM1][index][DSHOT_CCR3]=DSHOT_FRAME_SIZE-DMA2_Channel5->CNDTR-1;
      DSHOT_rx_cnt[DSHOT_TIM1][index][DSHOT_CCR4]=DSHOT_FRAME_SIZE-DMA2_Channel6->CNDTR-1;
      
      ModeTIM1_WAIT();
    }
  }
  
  if((TIM16->SR & TIM16->DIER) & TIM_SR_UIF)
  {
    if(DSHOT_Wait[DSHOT_TIM16])
    {
      ModeTIM16_TX();
    }
    else
    {
      ModeTIM16_WAIT();
    }
  }
}
//------------------------------------------------------------------------------

extern "C" void TIM1_TRG_COM_TIM17_IRQHandler()
{
  if((TIM17->SR & TIM17->DIER) & TIM_SR_UIF)
  {
    if(DSHOT_Wait[DSHOT_TIM17])
    {
      ModeTIM17_TX();
    }
    else
    {
      ModeTIM17_WAIT();
    }
  }
}
//------------------------------------------------------------------------------

bool DSHOT_Init(unsigned long Freq, unsigned long Timer, bool Bidirect)
{
  if(RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN)   return false;
  if(RCC->APB2ENR & RCC_APB2ENR_TIM1EN)   return false;
  if(RCC->APB1ENR1 & RCC_APB1ENR1_TIM2EN) return false;
  
  RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMAMUX1EN;
  RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
  RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM16EN | RCC_APB2ENR_TIM17EN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  
  GPIO_InitPin(GPIO_PIN_0  | GPIO_PORT_A | GPIO_ALTER | GPIO_PULLUP | GPIO_AF1);  // TIM2_CH1
  GPIO_InitPin(GPIO_PIN_1  | GPIO_PORT_A | GPIO_ALTER | GPIO_PULLUP | GPIO_AF1);  // TIM2_CH2
  GPIO_InitPin(GPIO_PIN_10 | GPIO_PORT_A | GPIO_ALTER | GPIO_PULLUP | GPIO_AF6);  // TIM1_CH3
  GPIO_InitPin(GPIO_PIN_11 | GPIO_PORT_A | GPIO_ALTER | GPIO_PULLUP | GPIO_AF11); // TIM1_CH4
  
  GPIO_InitPin(GPIO_PIN_12 | GPIO_PORT_A | GPIO_ALTER | GPIO_PULLUP | GPIO_AF1);  // TIM16_CH1
  GPIO_InitPin(GPIO_PIN_5  | GPIO_PORT_B | GPIO_ALTER | GPIO_PULLUP | GPIO_AF10); // TIM17_CH1
  
  DSHOT_Mode(Freq, Timer, Bidirect);
  
  for (int a = 0; a < DSHOT_BUF; a++) // Zero end in triple buffers
  {
    for (int b = DSHOT_FRAME_DATA*DSHOT_CCR; b < DSHOT_FRAME_SIZE*DSHOT_CCR; b++) DSHOT_tx_buf[DSHOT_TIM2][a][b]  = 0;
    for (int b = DSHOT_FRAME_DATA*DSHOT_CCR; b < DSHOT_FRAME_SIZE*DSHOT_CCR; b++) DSHOT_tx_buf[DSHOT_TIM1][a][b]  = 0;
    for (int b = DSHOT_FRAME_DATA*DSHOT_CCR; b < DSHOT_FRAME_SIZE*DSHOT_CCR; b++) DSHOT_tx_buf[DSHOT_TIM16][a][b] = 0;
    for (int b = DSHOT_FRAME_DATA*DSHOT_CCR; b < DSHOT_FRAME_SIZE*DSHOT_CCR; b++) DSHOT_tx_buf[DSHOT_TIM17][a][b] = 0;
  }
  
  TIM2->BDTR = TIM_BDTR_MOE;
  TIM1->BDTR = TIM_BDTR_MOE;
  TIM16->BDTR = TIM_BDTR_MOE;
  TIM17->BDTR = TIM_BDTR_MOE;
  
  DMAMUX1_Channel6->CCR = (60 << DMAMUX_CxCR_DMAREQ_ID_Pos); // TIM2_UP
  DMAMUX1_Channel7->CCR = (46 << DMAMUX_CxCR_DMAREQ_ID_Pos); // TIM1_UP
  
  DMAMUX1_Channel8->CCR = (56 << DMAMUX_CxCR_DMAREQ_ID_Pos); // TIM2_CH1
  DMAMUX1_Channel9->CCR = (57 << DMAMUX_CxCR_DMAREQ_ID_Pos); // TIM2_CH2
  
  DMAMUX1_Channel10->CCR = (44 << DMAMUX_CxCR_DMAREQ_ID_Pos); // TIM1_CH3
  DMAMUX1_Channel11->CCR = (45 << DMAMUX_CxCR_DMAREQ_ID_Pos); // TIM1_CH4
  
  DMAMUX1_Channel4->CCR = (83 << DMAMUX_CxCR_DMAREQ_ID_Pos); // TIM16_UP
  DMAMUX1_Channel5->CCR = (85 << DMAMUX_CxCR_DMAREQ_ID_Pos); // TIM17_UP
  
  // TX DMA TIM2
  TIM2->DCR = (0x0D << TIM_DCR_DBA_Pos) | (1 << TIM_DCR_DBL_Pos); // Burst CCR1 & CCR2
  DMA2_Channel1->CPAR = (unsigned long)&TIM2->DMAR;
  DMA2_Channel1->CCR |= DMA_CCR_DIR | DMA_CCR_MINC |  DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1 | DMA_CCR_TCIE;
  
  // TX DMA TIM1
  TIM1->DCR = (0x0F << TIM_DCR_DBA_Pos) | (1 << TIM_DCR_DBL_Pos); // Burst CCR3 & CCR4
  DMA2_Channel2->CPAR = (unsigned long)&TIM1->DMAR;
  DMA2_Channel2->CCR |= DMA_CCR_DIR | DMA_CCR_MINC |  DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_TCIE;
  
  // TX DMA TIM16
  DMA1_Channel5->CPAR = (unsigned long)&TIM16->CCR1;
  DMA1_Channel5->CCR |= DMA_CCR_DIR | DMA_CCR_MINC |  DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_TCIE;
  
  // TX DMA TIM17
  DMA1_Channel6->CPAR = (unsigned long)&TIM17->CCR1;
  DMA1_Channel6->CCR |= DMA_CCR_DIR | DMA_CCR_MINC |  DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_TCIE;
  
  // RX DMA TIM2
  DMA2_Channel3->CPAR = (unsigned long)&TIM2->CCR1;
  DMA2_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1;
  
  DMA2_Channel4->CPAR = (unsigned long)&TIM2->CCR2;
  DMA2_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1;
  
  // RX DMA TIM1
  DMA2_Channel5->CPAR = (unsigned long)&TIM1->CCR3;
  DMA2_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
  
  DMA2_Channel6->CPAR = (unsigned long)&TIM1->CCR4;
  DMA2_Channel6->CCR = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
  
  unsigned short zero[4]={0,0,0,0};
  DSHOT_SetCommand(zero);
  
  ModeTIM2_TX();
  ModeTIM1_TX();
  
  ModeTIM16_TX();
  ModeTIM17_TX();
  
  NVIC_SetPriority(DMA2_Channel1_IRQn, 0);
  NVIC_SetPriority(DMA2_Channel2_IRQn, 0);
  NVIC_SetPriority(DMA1_Channel5_IRQn, 0);
  NVIC_SetPriority(DMA1_Channel6_IRQn, 0);
  
  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0);
  NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 0);
  
  NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  
  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
  
  return true;
}
//------------------------------------------------------------------------------

bool DSHOT_Mode(unsigned long Freq, unsigned long Timer, bool Bidirect)
{
  if(Timer<100) Timer=100;
  if(Freq<150'000) Freq=150'000;
  if(Timer>Freq/75) Timer=Freq/75;
     
  DSHOT_Waiting = SystemCoreClock/DSHOT_FREQ_SCALE;
  DSHOT_Timer=SystemCoreClock/DSHOT_Waiting/Timer;
  
  if(DSHOT_Timer>65000) DSHOT_Timer=65000;
  if(DSHOT_Timer<1) DSHOT_Timer=1;
  
  DSHOT_PeriodTX=SystemCoreClock/Freq;
  DSHOT_PeriodRX=DSHOT_PeriodTX*DSHOT_FRAME_SIZE+5000; // +30us
  
  DSHOT_Bit_0 = DSHOT_PeriodTX*379/1024; // 37%
  DSHOT_Bit_1 = DSHOT_PeriodTX*767/1024; // 75%
  
  unsigned long mul = Bidirect ? 3 : 1;
  
  DSHOT_Shift = DSHOT_FRAME_SIZE*mul*DSHOT_PeriodTX/DSHOT_Waiting;
  if(DSHOT_Shift>=DSHOT_Timer) DSHOT_Shift=DSHOT_Timer-1;
  
  DSHOT_Bidirect=Bidirect;
}
//------------------------------------------------------------------------------
     
// Переключение в режим приема
static void ModeTIM2_RX() 
{
  TIM2->CR1 = 0;
  
  TIM2->CCER = 0;
  TIM2->ARR = DSHOT_PeriodRX;
  TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | (0x3 << TIM_CCMR1_IC1F_Pos) | TIM_CCMR1_CC2S_0 | (0x3 << TIM_CCMR1_IC2F_Pos); // CC1S = 01 (Input) & CC2S = 01 (Input)
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = ~TIM_SR_UIF;
  TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP; // Оба фронта
  TIM2->DIER = TIM_DIER_CC1DE | TIM_DIER_CC2DE | TIM_DIER_UIE;
  
  unsigned char index = buf_select(&DSHOT_rx_li[DSHOT_TIM2]);
  DSHOT_rx_sel[DSHOT_TIM2] = index;
  
  DMA2_Channel3->CCR &= ~DMA_CCR_EN;
  DMA2_Channel3->CMAR = (unsigned long)DSHOT_rx_buf[DSHOT_TIM2][index][DSHOT_CCR1]; // TIM2 | BUF | CCR1
  DMA2_Channel3->CNDTR = DSHOT_FRAME_SIZE;
  DMA2_Channel3->CCR |= DMA_CCR_EN;
  
  DMA2_Channel4->CCR &= ~DMA_CCR_EN;
  DMA2_Channel4->CMAR = (unsigned long)DSHOT_rx_buf[DSHOT_TIM2][index][DSHOT_CCR2]; // TIM2 | BUF | CCR2
  DMA2_Channel4->CNDTR = DSHOT_FRAME_SIZE;
  DMA2_Channel4->CCR |= DMA_CCR_EN;
  
  TIM2->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------

static void ModeTIM1_RX() 
{
  TIM1->CR1 = 0;
  
  TIM1->CCER = 0;
  TIM1->ARR = DSHOT_PeriodRX;
  TIM1->CCMR2 = TIM_CCMR2_CC3S_0 | (0x3 << TIM_CCMR2_IC3F_Pos) | TIM_CCMR2_CC4S_0 | (0x3 << TIM_CCMR2_IC4F_Pos); // CC3S = 01 (Input) & CC4S = 01 (Input)
  TIM1->EGR = TIM_EGR_UG;
  TIM1->SR = ~TIM_SR_UIF;
  TIM1->CCER = TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NP | TIM_CCER_CC4E | TIM_CCER_CC4P | TIM_CCER_CC4NP; // Оба фронта
  TIM1->DIER = TIM_DIER_CC3DE | TIM_DIER_CC4DE | TIM_DIER_UIE;
  
  unsigned char index = buf_select(&DSHOT_rx_li[DSHOT_TIM1]);
  DSHOT_rx_sel[DSHOT_TIM1] = index;
  
  DMA2_Channel5->CCR &= ~DMA_CCR_EN;
  DMA2_Channel5->CMAR = (unsigned long)DSHOT_rx_buf[DSHOT_TIM1][index][DSHOT_CCR3]; // TIM1 | BUF | CCR3
  DMA2_Channel5->CNDTR = DSHOT_FRAME_SIZE;
  DMA2_Channel5->CCR |= DMA_CCR_EN;
  
  DMA2_Channel6->CCR &= ~DMA_CCR_EN;
  DMA2_Channel6->CMAR = (unsigned long)DSHOT_rx_buf[DSHOT_TIM1][index][DSHOT_CCR4]; // TIM1 | BUF | CCR4
  DMA2_Channel6->CNDTR = DSHOT_FRAME_SIZE;
  DMA2_Channel6->CCR |= DMA_CCR_EN;
  
  TIM1->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------

// Переключение в режим передачи
static void ModeTIM2_TX()
{
  DSHOT_Wait[DSHOT_TIM2]=false;
  
  TIM2->CR1 = 0;
  if(DSHOT_Bidirect) TIM2->CCER = 0;
  TIM2->PSC = 0;
  TIM2->ARR = DSHOT_PeriodTX;
  TIM2->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM2->CCR1=TIM2->CCR2=0;
  TIM2->EGR = TIM_EGR_UG;
  if(DSHOT_Bidirect) TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC2E | TIM_CCER_CC2P;
  else               TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM2->DIER = TIM_DIER_UDE;
  
  unsigned char index=buf_lock(&DSHOT_tx_li[DSHOT_TIM2]);
  
  DMA2_Channel1->CCR &= ~DMA_CCR_EN;
  DMA2_Channel1->CMAR = (unsigned long)DSHOT_tx_buf[DSHOT_TIM2][index];
  DMA2_Channel1->CNDTR = DSHOT_FRAME_SIZE*DSHOT_CCR;
  DMA2_Channel1->CCR |= DMA_CCR_EN;
  
  TIM2->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------

static void ModeTIM1_TX()
{
  DSHOT_Wait[DSHOT_TIM1]=false;
  
  TIM1->CR1 = 0;
  if(DSHOT_Bidirect) TIM1->CCER = 0;
  TIM1->PSC = 0;
  TIM1->ARR = DSHOT_PeriodTX;
  TIM1->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
  TIM1->CCR3=TIM1->CCR4=0;
  TIM1->EGR = TIM_EGR_UG;
  if(DSHOT_Bidirect) TIM1->CCER = TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC4E | TIM_CCER_CC4P;
  else               TIM1->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
  TIM1->DIER = TIM_DIER_UDE;
  
  unsigned char index=buf_lock(&DSHOT_tx_li[DSHOT_TIM1]);
  
  DMA2_Channel2->CCR &= ~DMA_CCR_EN;
  DMA2_Channel2->CMAR = (unsigned long)DSHOT_tx_buf[DSHOT_TIM1][index];
  DMA2_Channel2->CNDTR = DSHOT_FRAME_SIZE*DSHOT_CCR;
  DMA2_Channel2->CCR |= DMA_CCR_EN;
  
  TIM1->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------
     
static void ModeTIM16_TX()
{
  DSHOT_Wait[DSHOT_TIM16]=false;
  
  TIM16->CR1 = 0;
  //if(DSHOT_Bidirect) TIM16->CCER = 0;
  TIM16->PSC = 0;
  TIM16->ARR = DSHOT_PeriodTX;
  TIM16->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
  TIM16->CCR1=0;
  TIM16->EGR = TIM_EGR_UG;
  //if(DSHOT_Bidirect) TIM16->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P;
  //else               TIM16->CCER = TIM_CCER_CC1E;
  TIM16->CCER = TIM_CCER_CC1E;
  TIM16->DIER = TIM_DIER_UDE;
  
  unsigned char index=buf_lock(&DSHOT_tx_li[DSHOT_TIM16]);
  
  DMA1_Channel5->CCR &= ~DMA_CCR_EN;
  DMA1_Channel5->CMAR = (unsigned long)DSHOT_tx_buf[DSHOT_TIM16][index];
  DMA1_Channel5->CNDTR = DSHOT_FRAME_SIZE;
  DMA1_Channel5->CCR |= DMA_CCR_EN;
  
  TIM16->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------
     
static void ModeTIM17_TX()
{
  DSHOT_Wait[DSHOT_TIM17]=false;
  
  TIM17->CR1 = 0;
  //if(DSHOT_Bidirect) TIM17->CCER = 0;
  TIM17->PSC = 0;
  TIM17->ARR = DSHOT_PeriodTX;
  TIM17->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
  TIM17->CCR1=0;
  TIM17->EGR = TIM_EGR_UG;
  //if(DSHOT_Bidirect) TIM17->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P;
  //else               TIM17->CCER = TIM_CCER_CC1E;
  TIM17->CCER = TIM_CCER_CC1E;
  TIM17->DIER = TIM_DIER_UDE;
  
  unsigned char index=buf_lock(&DSHOT_tx_li[DSHOT_TIM17]);
  
  DMA1_Channel6->CCR &= ~DMA_CCR_EN;
  DMA1_Channel6->CMAR = (unsigned long)DSHOT_tx_buf[DSHOT_TIM17][index];
  DMA1_Channel6->CNDTR = DSHOT_FRAME_SIZE;
  DMA1_Channel6->CCR |= DMA_CCR_EN;
  
  TIM17->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------

static void ModeTIM2_WAIT()
{
  DSHOT_Wait[DSHOT_TIM2]=true;
  
  buf_change(&DSHOT_rx_li[DSHOT_TIM2], DSHOT_rx_sel[DSHOT_TIM2]);
  
  TIM2->CR1 = 0;
  TIM2->PSC = DSHOT_Waiting - 1;
  TIM2->ARR = DSHOT_Timer - DSHOT_Shift;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = ~TIM_SR_UIF;
  TIM2->DIER = TIM_DIER_UIE;
  TIM2->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------

static void ModeTIM1_WAIT()
{
  DSHOT_Wait[DSHOT_TIM1]=true;
  
  buf_change(&DSHOT_rx_li[DSHOT_TIM1], DSHOT_rx_sel[DSHOT_TIM1]);
  
  TIM1->CR1 = 0;
  TIM1->PSC = DSHOT_Waiting - 1;
  TIM1->ARR = DSHOT_Timer - DSHOT_Shift;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->SR = ~TIM_SR_UIF;
  TIM1->DIER = TIM_DIER_UIE;
  TIM1->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------
     
static void ModeTIM16_WAIT()
{
  DSHOT_Wait[DSHOT_TIM16]=true;
  
  TIM16->CR1 = 0;
  TIM16->PSC = DSHOT_Waiting - 1;
  TIM16->ARR = DSHOT_Timer - DSHOT_Shift;
  TIM16->EGR = TIM_EGR_UG;
  TIM16->SR = ~TIM_SR_UIF;
  TIM16->DIER = TIM_DIER_UIE;
  TIM16->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------
     
static void ModeTIM17_WAIT()
{
  DSHOT_Wait[DSHOT_TIM17]=true;
  
  TIM17->CR1 = 0;
  TIM17->PSC = DSHOT_Waiting - 1;
  TIM17->ARR = DSHOT_Timer - DSHOT_Shift;
  TIM17->EGR = TIM_EGR_UG;
  TIM17->SR = ~TIM_SR_UIF;
  TIM17->DIER = TIM_DIER_UIE;
  TIM17->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------

static void ds_packet1(unsigned short throttle, bool tele, unsigned short* buff)
{
  unsigned short packet = (throttle << 5) | ((tele) << 4); 
  unsigned short checksum;
  
  if(DSHOT_Bidirect) checksum = ~(packet >> 4 ^ packet >> 8 ^ packet >> 12) & 0x0F;
  else               checksum =  (packet >> 4 ^ packet >> 8 ^ packet >> 12) & 0x0F;
  
  packet |= checksum;
  
  for (int a = 0; a < DSHOT_FRAME_DATA; a++) buff[a] = (packet & (0x8000 >> a)) ? DSHOT_Bit_1 : DSHOT_Bit_0;
}
//------------------------------------------------------------------------------
     
static void ds_packet2(unsigned short throttle, bool tele, unsigned short* buff, unsigned long shift)
{
  unsigned short packet = (throttle << 5) | ((tele) << 4); 
  unsigned short checksum;
  
  if(DSHOT_Bidirect) checksum = ~(packet >> 4 ^ packet >> 8 ^ packet >> 12) & 0x0F;
  else               checksum =  (packet >> 4 ^ packet >> 8 ^ packet >> 12) & 0x0F;
  
  packet |= checksum;
  
  for (int a = 0, b = shift; a < DSHOT_FRAME_DATA; a++, b+=2) buff[b] = (packet & (0x8000 >> a)) ? DSHOT_Bit_1 : DSHOT_Bit_0;
}
//------------------------------------------------------------------------------
     
void DSHOT_SetCommand(unsigned short Command[6])
{
  if(!Command) return;
  
  if(DSHOT_Comm[DSHOT_TIM2][DSHOT_CCR1]!=Command[0] || DSHOT_Comm[DSHOT_TIM2][DSHOT_CCR2]!=Command[1])
  {
    unsigned short c1 = DSHOT_Comm[DSHOT_TIM2][DSHOT_CCR1]=Command[0]; 
    unsigned short c2 = DSHOT_Comm[DSHOT_TIM2][DSHOT_CCR2]=Command[1];
    
    unsigned char next = buf_select(&DSHOT_tx_li[DSHOT_TIM2]);
    
    unsigned short* buf = DSHOT_tx_buf[DSHOT_TIM2][next];
    
    ds_packet2(c1 & 0x0FFF, c1 >> 15, buf, DSHOT_CCR1);
    ds_packet2(c2 & 0x0FFF, c2 >> 15, buf, DSHOT_CCR2);
    
    buf_change(&DSHOT_tx_li[DSHOT_TIM2], next);
  }
  
  if(DSHOT_Comm[DSHOT_TIM1][DSHOT_CCR3]!=Command[2] || DSHOT_Comm[DSHOT_TIM1][DSHOT_CCR4]!=Command[3])
  {
    unsigned short c3 = DSHOT_Comm[DSHOT_TIM1][DSHOT_CCR3]=Command[2]; 
    unsigned short c4 = DSHOT_Comm[DSHOT_TIM1][DSHOT_CCR4]=Command[3]; 
    
    unsigned char next = buf_select(&DSHOT_tx_li[DSHOT_TIM1]);
    
    unsigned short* buf = DSHOT_tx_buf[DSHOT_TIM1][next];
    
    ds_packet2(c3 & 0x0FFF, c3 >> 15, buf, DSHOT_CCR3);
    ds_packet2(c4 & 0x0FFF, c4 >> 15, buf, DSHOT_CCR4);
    
    buf_change(&DSHOT_tx_li[DSHOT_TIM1], next);
  }
  
  if(DSHOT_Comm[DSHOT_TIM16][DSHOT_CCR1]!=Command[4])
  {
    unsigned short c1 = DSHOT_Comm[DSHOT_TIM16][DSHOT_CCR1]=Command[4];
    
    unsigned char next = buf_select(&DSHOT_tx_li[DSHOT_TIM16]);
    
    unsigned short* buf = DSHOT_tx_buf[DSHOT_TIM16][next];
    
    ds_packet1(c1 & 0x0FFF, c1 >> 15, buf);
    
    buf_change(&DSHOT_tx_li[DSHOT_TIM16], next);
  }
  
  if(DSHOT_Comm[DSHOT_TIM17][DSHOT_CCR1]!=Command[5])
  {
    unsigned short c1 = DSHOT_Comm[DSHOT_TIM17][DSHOT_CCR1]=Command[5];
    
    unsigned char next = buf_select(&DSHOT_tx_li[DSHOT_TIM17]);
    
    unsigned short* buf = DSHOT_tx_buf[DSHOT_TIM17][next];
    
    ds_packet1(c1 & 0x0FFF, c1 >> 15, buf);
    
    buf_change(&DSHOT_tx_li[DSHOT_TIM17], next);
  }
}
//------------------------------------------------------------------------------

void DSHOT_GetERPM(unsigned short eRPM[4], bool Error[4])
{
  if(!eRPM) return;
  
  bool error;
  unsigned long erpm;
  unsigned char index;
  
  if(buf_new(&DSHOT_rx_li[DSHOT_TIM2]))
  {
    index = buf_lock(&DSHOT_rx_li[DSHOT_TIM2]);
    
    erpm = CalcERPM(DSHOT_rx_buf[DSHOT_TIM2][index][DSHOT_CCR1], DSHOT_FRAME_SIZE, DSHOT_rx_cnt[DSHOT_TIM2][index][DSHOT_CCR1]);
    error = erpm==-1;
    if(Error) Error[0]=error; 
    if(!error) DSHOT_ERPM[DSHOT_TIM2][DSHOT_CCR1]=erpm;
    eRPM[0]=DSHOT_ERPM[DSHOT_TIM2][DSHOT_CCR1];

    erpm = CalcERPM(DSHOT_rx_buf[DSHOT_TIM2][index][DSHOT_CCR2], DSHOT_FRAME_SIZE, DSHOT_rx_cnt[DSHOT_TIM2][index][DSHOT_CCR2]);
    error = erpm==-1;
    if(Error) Error[1]=error; 
    if(!error) DSHOT_ERPM[DSHOT_TIM2][DSHOT_CCR2]=erpm;
    eRPM[1]=DSHOT_ERPM[DSHOT_TIM2][DSHOT_CCR2];
    
  }
  
  if(buf_new(&DSHOT_rx_li[DSHOT_TIM1]))
  {
    index = buf_lock(&DSHOT_rx_li[DSHOT_TIM1]);
    
    erpm = CalcERPM(DSHOT_rx_buf[DSHOT_TIM1][index][DSHOT_CCR3], DSHOT_FRAME_SIZE, DSHOT_rx_cnt[DSHOT_TIM1][index][DSHOT_CCR3]);
    error = erpm==-1;
    if(Error) Error[2]=error; 
    if(!error) DSHOT_ERPM[DSHOT_TIM1][DSHOT_CCR3]=erpm;
    eRPM[2]=DSHOT_ERPM[DSHOT_TIM1][DSHOT_CCR3];

    erpm = CalcERPM(DSHOT_rx_buf[DSHOT_TIM1][index][DSHOT_CCR4], DSHOT_FRAME_SIZE, DSHOT_rx_cnt[DSHOT_TIM1][index][DSHOT_CCR4]);
    error = erpm==-1;
    if(Error) Error[3]=error; 
    if(!error) DSHOT_ERPM[DSHOT_TIM1][DSHOT_CCR4]=erpm;
    eRPM[3]=DSHOT_ERPM[DSHOT_TIM1][DSHOT_CCR4];
    
  }
}
//------------------------------------------------------------------------------
     
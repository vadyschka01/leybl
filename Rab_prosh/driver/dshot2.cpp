#include "stm32g4xx.h"

#include "gpio.h"

#include "dshot2.h"

#define DSHOT_FREQ_SCALE 10'000'000

#define DSHOT_FRAME_DATA 16 
#define DSHOT_FRAME_SIZE 20

#define DSHOT_COUNT 4

#define DSHOT_TIM   2

#define DSHOT_TIM2  0
#define DSHOT_TIM1  1

#define DSHOT_CCR  2
#define DSHOT_CCR1 0
#define DSHOT_CCR2 1
#define DSHOT_CCR3 0
#define DSHOT_CCR4 1

static bool DSHOT_Bidirect;

static unsigned short DSHOT_Shift, DSHOT_PeriodTX, DSHOT_PeriodRX, DSHOT_Bit_0, DSHOT_Bit_1;

static unsigned short DSHOT_TX_Buff[DSHOT_TIM][DSHOT_FRAME_SIZE*DSHOT_CCR]; // TIM | CCR1 + CCR2

#define DSHOT_RX_SAMP 4
#define DSHOT_RX_SIZE 32

static unsigned short DSHOT_RX_Buff[DSHOT_RX_SIZE*DSHOT_RX_SAMP];
static GPIO_TypeDef* DSHOT_RX_Port;
static unsigned long DSHOT_RX_Mode, DSHOT_TX_Mode;
static unsigned short DSHOT_RX_Index[DSHOT_COUNT];

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

static unsigned long CalcERPM(unsigned long code)
{
  code = code ^ (code>>1);
  
  const unsigned long mask = 0x1F; 

  unsigned long gcr1 = (code >> 0)  & mask;
  unsigned long gcr2 = (code >> 5)  & mask;
  unsigned long gcr3 = (code >> 10) & mask;
  unsigned long gcr4 = (code >> 15) & mask;
  
  gcr1=decodeGCR(gcr1);
  gcr2=decodeGCR(gcr2);
  gcr3=decodeGCR(gcr3);
  gcr4=decodeGCR(gcr4);
  
  unsigned long crc = (~(gcr2 ^ gcr3 ^ gcr4)) & 0x0F;

  bool valid = crc==gcr1;
  
  if (!valid || gcr1==0xFF || gcr2==0xFF || gcr3==0xFF || gcr4==0xFF) return -1;
  
  unsigned short data = (gcr4 << 8) | (gcr3 << 4) | gcr2;
  
  unsigned long e = (data >> 9) & 0x07; // Старшие 3 бита
  unsigned long m = data & 0x1FF; 
  
  unsigned long p = m<<e;
  
  if (p==0) return -1;
  
  return 1'000'000/p;
}
//------------------------------------------------------------------------------

static void Begin_ModeRX()
{
  TIM6->ARR  = 2300; // 20us
  TIM6->EGR  = TIM_EGR_UG;
  TIM6->SR   = 0;
  TIM6->DIER = TIM_DIER_UIE;
  TIM6->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------

extern "C" void DMA2_Channel1_IRQHandler()
{
  unsigned long sr = DMA2->ISR;
  if(sr & DMA_ISR_TCIF1)
  {
    DMA2->IFCR = DMA_IFCR_CTCIF1;
    
    Begin_ModeRX();
  }
}
//------------------------------------------------------------------------------

extern "C" void TIM6_DAC_IRQHandler()
{
  DSHOT_RX_Port->MODER &= DSHOT_RX_Mode;
  TIM6->ARR  = DSHOT_PeriodRX;
  TIM6->EGR  = TIM_EGR_UG;
  TIM6->DIER = TIM_DIER_UDE;
}
//------------------------------------------------------------------------------

     
static void Begin_ModeTX()
{
  DSHOT_RX_Port->MODER |= DSHOT_TX_Mode;
}
//------------------------------------------------------------------------------

extern "C" void DMA2_Channel3_IRQHandler()
{
  unsigned long sr = DMA2->ISR;
  if(sr & DMA_ISR_TCIF3)
  {
    DMA2->IFCR = DMA_IFCR_CTCIF3;
    TIM6->DIER = 0;
    
    Begin_ModeTX();
  }
}
//------------------------------------------------------------------------------

bool DSHOT_Mode(unsigned long Freq, bool Bidirect)
{
  if(Freq<150'000) Freq=150'000;
  
  DSHOT_PeriodTX=SystemCoreClock/Freq;
  DSHOT_PeriodRX=(DSHOT_PeriodTX-DSHOT_PeriodTX/4)/DSHOT_RX_SAMP;
  
  DSHOT_Bit_0 = DSHOT_PeriodTX*379/1024; // 37%
  DSHOT_Bit_1 = DSHOT_PeriodTX*767/1024; // 75%
  
  DSHOT_Bidirect=Bidirect;
}
//------------------------------------------------------------------------------

// Режим приема
static bool Init_ModeRX()
{
  if(RCC->APB1ENR1 & RCC_APB1ENR1_TIM6EN) return false;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
  
  if(DMAMUX1_Channel8->CCR & DMAMUX_CxCR_DMAREQ_ID_Msk) return false;
  DMAMUX1_Channel8->CCR = (8 << DMAMUX_CxCR_DMAREQ_ID_Pos); // TIM6_UP
  
  DMA2_Channel3->CCR  = 0;
  DMA2_Channel3->CPAR = (unsigned long)&GPIOA->IDR;
  DMA2_Channel3->CMAR = (unsigned long)DSHOT_RX_Buff;
  DMA2_Channel3->CCR  = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_TCIE;
  
  NVIC_SetPriority(DMA2_Channel3_IRQn, 0);
  NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  
  NVIC_SetPriority(TIM6_DAC_IRQn, 0);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
  
  return true;
}
//------------------------------------------------------------------------------
     
// Режим передачи
static void Init_TIM2_TX()
{
  TIM2->CR1 = 0;
  TIM2->SMCR = (6 << TIM_SMCR_SMS_Pos); // Slaver
  TIM2->PSC = 0;
  TIM2->ARR = DSHOT_PeriodTX;
  TIM2->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM2->CCR1=TIM2->CCR2=0;
  TIM2->EGR = TIM_EGR_UG;
  if(DSHOT_Bidirect) TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC2E | TIM_CCER_CC2P;
  else               TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM2->DIER = TIM_DIER_UDE;
  TIM2->DCR = (0x0D << TIM_DCR_DBA_Pos) | (1 << TIM_DCR_DBL_Pos); // Burst CCR1 & CCR2
  
  TIM2->BDTR = TIM_BDTR_MOE;
  
  DMAMUX1_Channel6->CCR = (60 << DMAMUX_CxCR_DMAREQ_ID_Pos); // TIM2_UP
  
  // TX DMA TIM2
  DMA2_Channel1->CPAR = (unsigned long)&TIM2->DMAR;
  DMA2_Channel1->CMAR = (unsigned long)DSHOT_TX_Buff[DSHOT_TIM2];
  DMA2_Channel1->CCR  = DMA_CCR_DIR | DMA_CCR_MINC |  DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1;
  
  if(DSHOT_Bidirect) DMA2_Channel1->CCR |= DMA_CCR_TCIE;
  
  NVIC_SetPriority(DMA2_Channel1_IRQn, 0);
  
  NVIC_EnableIRQ(DMA2_Channel1_IRQn);
}
//------------------------------------------------------------------------------
     
static void Init_TIM1_TX()
{
  TIM1->CR1 = 0;
  TIM1->CR2 |= (0x01 << TIM_CR2_MMS_Pos); // Master генерирует TRGO при включении (CEN=1)
  TIM1->PSC = 0;
  TIM1->ARR = DSHOT_PeriodTX;
  TIM1->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
  TIM1->CCR3=TIM1->CCR4=0;
  TIM1->EGR = TIM_EGR_UG;
  if(DSHOT_Bidirect) TIM1->CCER = TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC4E | TIM_CCER_CC4P;
  else               TIM1->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
  TIM1->DIER = TIM_DIER_UDE;
  TIM1->DCR = (0x0F << TIM_DCR_DBA_Pos) | (1 << TIM_DCR_DBL_Pos); // Burst CCR3 & CCR4
  
  TIM1->BDTR = TIM_BDTR_MOE;
 
  DMAMUX1_Channel7->CCR = (46 << DMAMUX_CxCR_DMAREQ_ID_Pos); // TIM1_UP

  // TX DMA TIM1
  DMA2_Channel2->CPAR = (unsigned long)&TIM1->DMAR;
  DMA2_Channel2->CMAR = (unsigned long)DSHOT_TX_Buff[DSHOT_TIM1];
  DMA2_Channel2->CCR |= DMA_CCR_DIR | DMA_CCR_MINC |  DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
}
//------------------------------------------------------------------------------
     
bool DSHOT_Init(unsigned long Freq, bool Bidirect)
{
  if(RCC->APB2ENR & RCC_APB2ENR_TIM1EN)   return false;
  if(RCC->APB1ENR1 & RCC_APB1ENR1_TIM2EN) return false;
  
  RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  
  RCC->AHB1ENR  |= RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMAMUX1EN;
  
  RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
  
  GPIO_InitPin(GPIO_PIN_0  | GPIO_PORT_A | GPIO_ALTER | GPIO_PULLUP | GPIO_AF1);  // TIM2_CH1
  GPIO_InitPin(GPIO_PIN_1  | GPIO_PORT_A | GPIO_ALTER | GPIO_PULLUP | GPIO_AF1);  // TIM2_CH2
  GPIO_InitPin(GPIO_PIN_10 | GPIO_PORT_A | GPIO_ALTER | GPIO_PULLUP | GPIO_AF6);  // TIM1_CH3
  GPIO_InitPin(GPIO_PIN_11 | GPIO_PORT_A | GPIO_ALTER | GPIO_PULLUP | GPIO_AF11); // TIM1_CH4
  
  DSHOT_RX_Port=GPIOA;
  DSHOT_TX_Mode=GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1 | GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1;
  DSHOT_RX_Mode=~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE10 | GPIO_MODER_MODE11);
  
  DSHOT_RX_Index[0]=1<<GPIO_PIN_0;
  DSHOT_RX_Index[1]=1<<GPIO_PIN_1;
  DSHOT_RX_Index[2]=1<<GPIO_PIN_10;
  DSHOT_RX_Index[3]=1<<GPIO_PIN_11;
  
  DSHOT_Mode(Freq, Bidirect);
  
  for (int a = 0; a < DSHOT_TIM; a++) // Zero end in buffer
    for (int b = DSHOT_FRAME_DATA*DSHOT_CCR; b < DSHOT_FRAME_SIZE*DSHOT_CCR; b++)
      DSHOT_TX_Buff[a][b] = 0;

  if(Bidirect) Init_ModeRX();
  
  Init_TIM2_TX();
  Init_TIM1_TX();
  
  return true;
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
     
bool DSHOT_SetCommand(unsigned short Command[4], bool Forced)
{
  if(!Command) return false;
  
  if(!Forced)
  {
    if(DMA2_Channel1->CNDTR || DMA2_Channel2->CNDTR) return false;
    if(DSHOT_Bidirect && DMA2_Channel3->CNDTR) return false;
  }
  
  TIM2->CR1 = TIM1->CR1 = TIM6->CR1 = 0;
  TIM2->DIER = TIM1->DIER = TIM6->DIER = 0;
  
  if(Forced) Begin_ModeTX();
  
  ds_packet2(Command[0] & 0x0FFF, Command[0] >> 15, DSHOT_TX_Buff[DSHOT_TIM2], DSHOT_CCR1);
  ds_packet2(Command[1] & 0x0FFF, Command[1] >> 15, DSHOT_TX_Buff[DSHOT_TIM2], DSHOT_CCR2);
  
  ds_packet2(Command[2] & 0x0FFF, Command[2] >> 15, DSHOT_TX_Buff[DSHOT_TIM1], DSHOT_CCR3);
  ds_packet2(Command[3] & 0x0FFF, Command[3] >> 15, DSHOT_TX_Buff[DSHOT_TIM1], DSHOT_CCR4);
  
  DMA2_Channel1->CCR  &= ~DMA_CCR_EN;
  DMA2_Channel1->CNDTR =  DSHOT_FRAME_SIZE*DSHOT_CCR;
  DMA2_Channel1->CCR  |=  DMA_CCR_EN;
  
  DMA2_Channel2->CCR  &= ~DMA_CCR_EN;
  DMA2_Channel2->CNDTR =  DSHOT_FRAME_SIZE*DSHOT_CCR;
  DMA2_Channel2->CCR  |=  DMA_CCR_EN;
  
  TIM2->DIER = TIM1->DIER = TIM_DIER_UDE;
  
  if(DSHOT_Bidirect)
  {
    DMA2_Channel3->CCR  &= ~DMA_CCR_EN;
    DMA2_Channel3->CNDTR =  DSHOT_RX_SIZE*DSHOT_RX_SAMP;
    DMA2_Channel3->CCR  |=  DMA_CCR_EN;
  }
  
  TIM1->CR1 = TIM_CR1_CEN; // TIM1(master) & TIM2(slaver)
  
  return true;
}
//------------------------------------------------------------------------------

bool DSHOT_GetERPM(unsigned short eRPM[4], bool Error[4])
{
  if(!eRPM) return false;
  
  struct
  {
    bool prev=true;
    unsigned long len=0;
    unsigned long code=0;
    unsigned long count=0;
  }data[DSHOT_COUNT];
  
  for(int a=1; a<DSHOT_RX_SIZE*DSHOT_RX_SAMP-1; a++)
  {
    unsigned short* buf = DSHOT_RX_Buff;
    unsigned short filt = (buf[a-1] & buf[a]) | (buf[a] & buf[a+1]) | (buf[a-1] & buf[a+1]);
    
    
    for(int b=0; b<DSHOT_COUNT; b++)
    {
      bool bit = filt & DSHOT_RX_Index[b];
      bool prv = data[b].prev;
      
      if(prv!=bit)
      {
        unsigned long cnt = data[b].len ? data[b].count : 0;
        
        if(cnt>=DSHOT_RX_SAMP*3-2) 
        {
          data[b].len+=3;
          data[b].code<<=3;
          if(prv) data[b].code|=0x07;
        }
        else if(cnt>=DSHOT_RX_SAMP*2-2) 
        {
          data[b].len+=2;
          data[b].code<<=2;
          if(prv) data[b].code|=0x03;
        }
        else
        {
          data[b].len+=1;
          data[b].code<<=1;
          if(prv) data[b].code|=0x01;
        }

        data[b].prev=bit;
        data[b].count=0;
      }
      else
      {
        data[b].count++;
      }
    }
    
  }
  
  for(int a=0; a<4; a++)
  {
    if(data[a].len==21) 
    {
      data[a].code<<=1;
      data[a].code|=1;
    }
    else if(data[a].len==20) 
    {
      data[a].code<<=2;
      data[a].code|=3;
    }
     
    short erpm = CalcERPM(data[a].code);
    if(Error) 
    {
      Error[a]=erpm==-1;
      if(Error[a] && !a) 
        int z=0;
    }
    eRPM[a] = erpm;
  }
}
//------------------------------------------------------------------------------

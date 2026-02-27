#include "stm32g4xx.h"

#include "tim.h"

static ProcTIM TIM6_Proc1 = 0;
static ProcTIM TIM6_Proc2 = 0;
static ProcTIM TIM7_Proc = 0;

extern "C" void TIM6_DAC_IRQHandler()
{
  static int proc=0;
  TIM6->SR = 0;
  
  
  if(proc>3) 
  {
    TIM6_Proc2();
    proc=0;
  }
  else proc++;
  
  TIM6_Proc1();
}
//------------------------------------------------------------------------------

extern "C" void TIM7_IRQHandler()
{
  TIM7->SR = 0;
  TIM7_Proc();
}
//------------------------------------------------------------------------------

void TIM6_Init(long Priority, unsigned long Freq, const ProcTIM& Proc1, const ProcTIM& Proc2)
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;

  TIM6->CR1 = 0;
  TIM6->ARR = 1000 - 1;
  TIM6->PSC = (SystemCoreClock / 1000 / Freq) - 1;
  TIM6->DIER = TIM_DIER_UIE;

  TIM6_Proc1 = Proc1;
  TIM6_Proc2 = Proc2;
  
  NVIC_SetPriority(TIM6_DAC_IRQn, Priority);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
}
//------------------------------------------------------------------------------

void TIM7_Init(long Priority, unsigned long Freq, const ProcTIM& Proc)
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;

  TIM7->CR1 = 0;
  TIM7->ARR = 1000 - 1;
  TIM7->PSC = (SystemCoreClock / 1000 / Freq) - 1;
  TIM7->DIER = TIM_DIER_UIE;

  TIM7_Proc = Proc;
  
  NVIC_SetPriority(TIM7_IRQn, Priority);
  NVIC_EnableIRQ(TIM7_IRQn);
}
//------------------------------------------------------------------------------

void TIM6_Enable()
{
  TIM6->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------

void TIM7_Enable()
{
  TIM7->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------

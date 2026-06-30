#include "stm32g4xx.h"

#include "tim.h"

static ProcTIM TIM7_Proc1 = 0;
static ProcTIM TIM7_Proc2 = 0;
static ProcTIM TIM7_Proc3 = 0;

extern "C" void TIM7_IRQHandler()
{
  static int proc2=1, proc3=1;
  TIM7->SR = 0;
  
  TIM7_Proc1();
  
  if(proc2>=5) 
  {
    TIM7_Proc2();
    proc2=1;
  }
  else proc2++;
  
  if(proc3>=20) 
  {
    TIM7_Proc3();
    proc3=1;
  }
  else proc3++;
}
//------------------------------------------------------------------------------

void TIM7_Init(long Priority, unsigned long Freq, const ProcTIM& Proc1, const ProcTIM& Proc2, const ProcTIM& Proc3)
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;

  TIM7->CR1 = 0;
  TIM7->ARR = 1000 - 1;
  TIM7->PSC = (SystemCoreClock / 1000 / Freq) - 1;
  TIM7->DIER = TIM_DIER_UIE;

  TIM7_Proc1 = Proc1;
  TIM7_Proc2 = Proc2;
  TIM7_Proc3 = Proc3;
  
  NVIC_SetPriority(TIM7_IRQn, Priority);
  NVIC_EnableIRQ(TIM7_IRQn);
}
//------------------------------------------------------------------------------

void TIM7_Enable()
{
  TIM7->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------

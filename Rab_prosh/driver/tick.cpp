#include "stm32g4xx.h"

#include "tim.h"

static unsigned long Tick = 0;

extern "C" void SysTick_Handler()
{
  Tick++;
}
//------------------------------------------------------------------------------

void TICK_Init()
{
  SysTick->LOAD = SystemCoreClock / 1000;
  SysTick->VAL = 0UL;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

  NVIC_SetPriority(SysTick_IRQn, 0);
}
//------------------------------------------------------------------------------

unsigned long TICK_GetCount() // ms
{
  return Tick;
}
//------------------------------------------------------------------------------

float TICK_GetTime() // s
{
  return ((float)Tick)/1000.0f;
}
//------------------------------------------------------------------------------

void TICK_Delay(unsigned long us)
{
  long tick=SysTick->VAL;
  long wait=us*(SystemCoreClock/1000000);
  
  long load = SysTick->LOAD;
  
  long val;
  
  do
  {
    val=SysTick->VAL;
    
    if(val>tick) val-=load;
    
  } while(tick-val < wait);
}
//------------------------------------------------------------------------------

#include "stm32g4xx.h"

#include "gpio.h"

#include "pwm.h"

bool PWM_Enable=false;

inline static unsigned short Mid(unsigned short Begin, unsigned short Val, unsigned short Min, unsigned short Max)
{
  if(Val<Min) return Min;
  if(Val>Max) return Max;
  return Val;
}

static void InitTIM(TIM_TypeDef* TIM, unsigned long Freq, unsigned long CCER)
{
  TIM->ARR = (1000000/Freq) - 1;
  TIM->PSC = (SystemCoreClock / (1000000/Freq) / Freq) - 1;
  TIM->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
  TIM->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;
  TIM->CCER = CCER;
  TIM->BDTR = TIM_BDTR_MOE;
  TIM->CCR1 = 0;
  TIM->CCR2 = 0;
  TIM->CCR3 = 0;
  TIM->CCR4 = 0;
  TIM->CR1 = TIM_CR1_CEN;
}
//------------------------------------------------------------------------------

void PWM_Init(unsigned long Freq)
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM16EN | RCC_APB2ENR_TIM17EN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

  GPIO_InitPin(GPIO_PIN_0  | GPIO_PORT_A | GPIO_ALTER | GPIO_AF1);  // TIM2_CH1
  GPIO_InitPin(GPIO_PIN_1  | GPIO_PORT_A | GPIO_ALTER | GPIO_AF1);  // TIM2_CH2
  GPIO_InitPin(GPIO_PIN_10 | GPIO_PORT_A | GPIO_ALTER | GPIO_AF6);  // TIM1_CH3
  GPIO_InitPin(GPIO_PIN_11 | GPIO_PORT_A | GPIO_ALTER | GPIO_AF11); // TIM1_CH4
  
  GPIO_InitPin(GPIO_PIN_12 | GPIO_PORT_A | GPIO_ALTER | GPIO_AF1);  // TIM16_CH1
  GPIO_InitPin(GPIO_PIN_5  | GPIO_PORT_B | GPIO_ALTER | GPIO_AF10); // TIM17_CH1
  
  InitTIM(TIM2,  Freq, TIM_CCER_CC1E | TIM_CCER_CC2E);
  InitTIM(TIM1,  Freq, TIM_CCER_CC3E | TIM_CCER_CC4E);
  InitTIM(TIM16, Freq, TIM_CCER_CC1E);
  InitTIM(TIM17, Freq, TIM_CCER_CC1E);
}
//------------------------------------------------------------------------------

static void Stop()
{
  PWM_Enable=false;
  
  TIM1->CCER=0;
  TIM2->CCER=0;
  TIM16->CCER=0;
  TIM17->CCER=0;
}
//------------------------------------------------------------------------------

void PWM_SetQuad(unsigned short Pow[4], unsigned short Min, unsigned short Max)
{
  if(!Pow) { Stop(); return; }
  TIM1->CCR4 = Mid(Pow[0] ? 1000 : 900, Pow[0], Min, Max);
  TIM2->CCR2 = Mid(Pow[1] ? 1000 : 900, Pow[1], Min, Max);
  TIM2->CCR1 = Mid(Pow[2] ? 1000 : 900, Pow[2], Min, Max);
  TIM1->CCR3 = Mid(Pow[3] ? 1000 : 900, Pow[3], Min, Max);
  
  bool en=false;
  for(int a=0; a<4; a++) if(Pow[a]>1050) en=true;
  PWM_Enable=en;
}
//------------------------------------------------------------------------------

void PWM_SetHexa(unsigned short Pow[6], unsigned short Min, unsigned short Max)
{
  if(!Pow) { Stop(); return; }
  TIM16->CCR1 = Mid(Pow[0] ? 1000 : 900, Pow[0], Min, Max); // D
  TIM17->CCR1 = Mid(Pow[1] ? 1000 : 900, Pow[1], Min, Max); // LU
  TIM2->CCR1  = Mid(Pow[2] ? 1000 : 900, Pow[2], Min, Max); // U
  TIM1->CCR3  = Mid(Pow[3] ? 1000 : 900, Pow[3], Min, Max); // RD
  TIM1->CCR4  = Mid(Pow[4] ? 1000 : 900, Pow[4], Min, Max); // RU
  TIM2->CCR2  = Mid(Pow[5] ? 1000 : 900, Pow[5], Min, Max); // LD
  
  bool en=false;
  for(int a=0; a<4; a++) if(Pow[a]>1050) en=true;
  PWM_Enable=en;
}
//------------------------------------------------------------------------------

void PWM_SetAll(unsigned short Pow)
{
  if(Pow) Pow+=1000;
  else Pow=900;
  
  TIM16->CCR1 = Pow;
  TIM17->CCR1 = Pow;
  TIM2->CCR1  = Pow;
  TIM1->CCR3  = Pow;
  TIM1->CCR4  = Pow;
  TIM2->CCR2  = Pow;
  
  bool en=false;
  if(Pow>1050) en=true;
  PWM_Enable=en;
}
//------------------------------------------------------------------------------

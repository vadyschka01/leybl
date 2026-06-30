#include "stm32g4xx.h"

#include "adc.h"

#include "gpio.h"

void ADC_Init()
{
  RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
  
  GPIO_InitPin(GPIO_PIN_0 | GPIO_PORT_B | GPIO_ANALOG);
  
  ADC12_COMMON->CCR |= ADC_CCR_CKMODE_1;
  //---
  ADC1->CR = ADC_CR_ADVREGEN;
  
  unsigned long wait = ((10 * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait--) {}
  
  ADC1->CR |= ADC_CR_ADCAL;
  
  while(ADC1->CR & ADC_CR_ADCAL) {}
  
  wait = 64;
  while(wait--) {}
  
  ADC1->ISR = ADC_ISR_ADRDY;
  ADC1->CR |= ADC_CR_ADEN;
  while(!(ADC1->ISR & ADC_ISR_ADRDY)) {}
  ADC1->ISR = ADC_ISR_ADRDY;
}
//-------------------------------------------------------------

static unsigned long ADC_GetValue(unsigned long I_Channel)
{
  ADC1->SQR1 = I_Channel<<6;
  
  unsigned long wait = 64;
  while(wait--) {}
  
  ADC1->ISR |= ADC_ISR_EOC;
  ADC1->CR |= ADC_CR_ADSTART;
  
  while(!(ADC1->ISR & ADC_ISR_EOC)) { } 
  
  return ADC1->DR;
}
//-------------------------------------------------------------

unsigned long ADC_GetVolt()
{
  unsigned long v = ADC_GetValue(15);
  return (v*10921)/1024;
}
//-------------------------------------------------------------

#include "stm32g4xx.h"

#include "gpio.h"

void GPIO_InitPin(unsigned long I_Pin)
{
  unsigned long port = (I_Pin & 0x000000F0UL) >> 4;
  GPIO_TypeDef* gpio = (GPIO_TypeDef*)(((unsigned char*)GPIOA) + (port * 0x0400));
  unsigned long rcc = 1UL << port;
  unsigned long pin = I_Pin & 0x0000000FUL;
  unsigned long af = (I_Pin & 0x0F000000UL) >> 24;
  unsigned long pupd = (I_Pin & 0x00F00000UL) >> 20;
  unsigned long ospeed = (I_Pin & 0x000F0000UL) >> 16;
  unsigned long mode = (I_Pin & 0x0000F000UL) >> 12;
  unsigned long otype = (I_Pin & 0x00000100UL) >> 8;
  unsigned long set = (I_Pin & 0x00000200UL) >> 9;
  //---
  if (!(RCC->AHB2ENR & rcc)) RCC->AHB2ENR |= rcc;
  //---
  gpio->AFR[pin >> 3] &= ~(0x0000000FUL << ((pin & 0x07) * 4));
  gpio->AFR[pin >> 3] |= af << ((pin & 0x07) * 4);
  //---
  gpio->OSPEEDR &= ~(0x00000003UL << (pin * 2));
  gpio->OSPEEDR |= ospeed << (pin * 2);
  //---
  gpio->OTYPER &= ~(0x00000001UL << pin);
  gpio->OTYPER |= otype << pin;
  //---
  gpio->PUPDR &= ~(0x00000003UL << (pin * 2));
  gpio->PUPDR |= pupd << (pin * 2);
  //---
  gpio->BSRR = 1 << (set ? pin : pin+16);
  //---
  gpio->MODER &= ~(0x00000003UL << (pin * 2));
  gpio->MODER |= mode << (pin * 2);
}
//------------------------------------------------------------------------------

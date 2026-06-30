#include "stm32g4xx.h"

#include "gpio.h"

#include "spi.h"

static void Init(SPI_TypeDef* SPI)
{
  SPI->CR1 = SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_SSI | SPI_CR1_SSM | (SPI_CR1_BR_2 |SPI_CR1_BR_1 | SPI_CR1_BR_0) | SPI_CR1_MSTR;
  SPI->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_FRXTH;
  SPI->CR1 |= SPI_CR1_SPE;
}
//------------------------------------------------------------------------------

void SPI1_Init()
{
  if(RCC->APB2ENR & RCC_APB2ENR_SPI1EN) return;

  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  GPIO_InitPin(GPIO_PIN_5 | GPIO_PORT_A | GPIO_ALTER | GPIO_AF5 | GPIO_OSPEED_HIGH); // SCK
  GPIO_InitPin(GPIO_PIN_6 | GPIO_PORT_A | GPIO_ALTER | GPIO_AF5 | GPIO_OSPEED_HIGH); // MISO
  GPIO_InitPin(GPIO_PIN_7 | GPIO_PORT_A | GPIO_ALTER | GPIO_AF5 | GPIO_OSPEED_HIGH); // MOSI
  
  GPIO_InitPin(GPIO_PIN_4  | GPIO_PORT_A | GPIO_OUTPUT | GPIO_OSPEED_HIGH | GPIO_SET); // NCS -> PMW3901

  Init(SPI1);
}
//------------------------------------------------------------------------------

static void Trans(SPI_TypeDef* SPI, const void* WriteData, void* ReadData, unsigned long Size)
{
  unsigned char* dr=(unsigned char*)ReadData;
  unsigned char* dw=(unsigned char*)WriteData;
  
  volatile char& DR = *((volatile char*)&(SPI->DR));

  while (Size--)
  {
    while (!(SPI->SR & SPI_SR_TXE)) { asm volatile("NOP"); }
    if (dw) DR = *(dw++);
    else DR = 0x00;

    while (!(SPI->SR & SPI_SR_RXNE)) { asm volatile("NOP"); }
    if (dr) *(dr++) = DR;
    else volatile char t = DR;
  }

  while (SPI->SR & SPI_SR_BSY) { asm volatile("NOP"); }
}
//------------------------------------------------------------------------------

void SPI1_CS_OF(bool En)
{
  if(En) 
  { 
    GPIOA->BSRR = GPIO_BSRR_BR_4;
    for (unsigned long a = 0; a < 21; a++) { asm volatile("NOP"); }
  }
  else
  {
    for (unsigned long a = 0; a < 21; a++) { asm volatile("NOP"); }
    GPIOA->BSRR = GPIO_BSRR_BS_4;
  }
  
}
//------------------------------------------------------------------------------

void SPI1_Transfer(const void* WriteData, void* ReadData, unsigned long Size)
{
  Trans(SPI1, WriteData, ReadData, Size);
}
//------------------------------------------------------------------------------

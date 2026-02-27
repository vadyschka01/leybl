#include "stm32g4xx.h"

#include <string.h>

#include "gpio.h"

#include "i2c.h"

struct I2C_Data
{
  I2C_TypeDef* I2C;
  
  IRQn_Type IRQn;
  
  I2C_Request *Head, *Device;
    
  unsigned char Index;
};

static I2C_Data I2C1_Data = { I2C1, I2C1_EV_IRQn, 0, 0, 0 };
static I2C_Data I2C2_Data = { I2C2, I2C2_EV_IRQn, 0, 0, 0 };

static void EV_IRQHandler(I2C_Data& I2C)
{
  if((I2C.I2C->CR1 & I2C_CR1_TXIE) && (I2C.I2C->ISR & I2C_ISR_TXE))
  {
    const I2C_Request& device=*I2C.Device;
    
    I2C.I2C->TXDR = device.Buffer[I2C.Index++];
    if(I2C.Index<device.Write) return;
    I2C.I2C->CR1 = I2C_CR1_TCIE | I2C_CR1_PE;
    I2C.Index=0;
    return;
  }
  //---
  if((I2C.I2C->CR1 & I2C_CR1_TCIE) && (I2C.I2C->ISR & I2C_ISR_TC))
  {
    const I2C_Request& device=*I2C.Device;
    
    if(device.Read)
    {
      I2C.I2C->CR2 &= ~I2C_CR2_NBYTES_Msk;
      I2C.I2C->CR2 |=  I2C_CR2_START | I2C_CR2_RD_WRN | (((unsigned long)device.Read)<<I2C_CR2_NBYTES_Pos);
      I2C.I2C->CR1  =  I2C_CR1_RXIE | I2C_CR1_PE;
    }
    else
    {
      I2C.I2C->CR2 |=  I2C_CR2_STOP;
      I2C.I2C->CR1  =  I2C_CR1_STOPIE | I2C_CR1_PE;
    }
    return;
  }
  //---
  if((I2C.I2C->CR1 & I2C_CR1_RXIE) && (I2C.I2C->ISR & I2C_ISR_RXNE))
  {
    const I2C_Request& device=*I2C.Device;
    
    device.Buffer[I2C.Index++] = I2C.I2C->RXDR;
    if(I2C.Index<device.Read) return;
    I2C.I2C->CR1  =  I2C_CR1_STOPIE | I2C_CR1_PE;
    I2C.I2C->CR2 |=  I2C_CR2_STOP;
    I2C.Index=0;
    return;
  }
  //---
  if((I2C.I2C->CR1 & I2C_CR1_STOPIE) && (I2C.I2C->ISR & I2C_ISR_STOPF))
  {
    I2C_Request& device=*I2C.Device;
    
    I2C.I2C->ICR = I2C_ICR_STOPCF;
    I2C.I2C->CR1 = I2C_CR1_PE;
    I2C.I2C->CR2 &= ~(I2C_CR2_SADD_Msk | I2C_CR2_RD_WRN | I2C_CR2_NBYTES_Msk);
    unsigned char addr=device.Address;
    device.Address=0;
    
    I2C.Device=device.Next;
    
    if(device.CallbackProc) device.CallbackProc(addr, device.Buffer, device.Read);
  }
  //---
  if(I2C.I2C->CR1 == I2C_CR1_PE)
  {
    if(!I2C.Device)
    {
      do
      {
        I2C.Device = (I2C_Request*)__LDREXW((volatile unsigned int*)&I2C.Head);
      }while(__STREXW(0, (volatile unsigned int*)&I2C.Head));
      
      if(!I2C.Device) return;
    }
    
    const I2C_Request& device=*I2C.Device;
    
    if(!(I2C.I2C->CR2 & I2C_CR2_SADD_Msk))
    {
      I2C.I2C->CR2 &= ~(I2C_CR2_SADD_Msk | I2C_CR2_RD_WRN | I2C_CR2_NBYTES_Msk);
      if(device.Write)
      {
        I2C.I2C->CR2 |= I2C_CR2_START | (device.Address << (I2C_CR2_SADD_Pos + 1)) | (((unsigned long)device.Write)<<I2C_CR2_NBYTES_Pos);
        I2C.I2C->CR1 = I2C_CR1_TXIE | I2C_CR1_PE;
      }
      else 
      {
        I2C.I2C->CR2 |= I2C_CR2_START | (device.Address << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_RD_WRN | (((unsigned long)device.Read)<<I2C_CR2_NBYTES_Pos);
        I2C.I2C->CR1 = I2C_CR1_RXIE | I2C_CR1_PE;
      }
    }
    return;
  }
}
//------------------------------------------------------------------------------

extern "C" void I2C1_EV_IRQHandler()
{
  EV_IRQHandler(I2C1_Data);
}
//------------------------------------------------------------------------------

extern "C" void I2C2_EV_IRQHandler()
{
  EV_IRQHandler(I2C2_Data);
}
//------------------------------------------------------------------------------

static void Init(I2C_TypeDef* I2C, IRQn_Type IRQn)
{
  //I2C->TIMINGR = 0x00303D5BUL; // 100kHz
  I2C->TIMINGR = 0x10802D9BUL; // 400kHz
  
  I2C->CR1 = I2C_CR1_PE;

  while(I2C->ISR & I2C_ISR_BUSY) { }
  
  NVIC_SetPriority(IRQn, 1);
  NVIC_EnableIRQ(IRQn);
}
//------------------------------------------------------------------------------

static bool I2C_CheckDevice(I2C_TypeDef* I2C, unsigned char Address) 
{
    if (I2C->ISR & I2C_ISR_BUSY) return false; 

    I2C->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;

    I2C->CR2 = (Address << 1) | I2C_CR2_AUTOEND | I2C_CR2_START;

    uint32_t timeout = 100000;
    
    while (!(I2C->ISR & I2C_ISR_STOPF)) 
    {
        if ((timeout--) == 0) return false;
    }

    bool device_present = false;
    
    if (I2C->ISR & I2C_ISR_NACKF) 
    {
        I2C->ICR = I2C_ICR_NACKCF;
        device_present = false;
    } 
    else device_present = true;

    I2C->ICR = I2C_ICR_STOPCF;
    I2C->CR2 = 0;

    return device_present;
}
//------------------------------------------------------------------------------

void I2C2_Init()
{
  if (RCC->APB1ENR1 & RCC_APB1ENR1_I2C2EN) return;

  RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  GPIO_InitPin(GPIO_PIN_8 | GPIO_PORT_A | GPIO_ALTER | GPIO_AF4 | GPIO_OSPEED_HIGH | GPIO_OPENDRAIN | GPIO_PULLUP);
  GPIO_InitPin(GPIO_PIN_9 | GPIO_PORT_A | GPIO_ALTER | GPIO_AF4 | GPIO_OSPEED_HIGH | GPIO_OPENDRAIN | GPIO_PULLUP);

  Init(I2C2, I2C2_EV_IRQn);
}
//------------------------------------------------------------------------------

void I2C1_Init()
{
  if (RCC->APB1ENR1 & RCC_APB1ENR1_I2C1EN) return;

  RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

  GPIO_InitPin(GPIO_PIN_8 | GPIO_PORT_B | GPIO_ALTER | GPIO_AF4 | GPIO_OSPEED_HIGH | GPIO_OPENDRAIN | GPIO_PULLUP);
  GPIO_InitPin(GPIO_PIN_9 | GPIO_PORT_B | GPIO_ALTER | GPIO_AF4 | GPIO_OSPEED_HIGH | GPIO_OPENDRAIN | GPIO_PULLUP);

  Init(I2C1, I2C1_EV_IRQn);
}
//------------------------------------------------------------------------------

static void Stop(I2C_TypeDef* I2C)
{
  I2C->CR2 |= I2C_CR2_STOP;
  while (!(I2C->ISR & I2C_ISR_STOPF)) { asm volatile("NOP"); }
  I2C->ICR = I2C_ICR_STOPCF;
  I2C->CR2 = 0;
}
//------------------------------------------------------------------------------

static void Read(I2C_TypeDef* I2C, unsigned char Address, unsigned char* Data, unsigned char Size)
{
  I2C->CR2 &= ~(I2C_CR2_SADD_Msk | I2C_CR2_NBYTES_Msk);
  I2C->CR2 |= (Address << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_RD_WRN | (((unsigned long)Size)<<I2C_CR2_NBYTES_Pos);
  I2C->CR2 |= I2C_CR2_START;
  
  while (Size--)
  {
    while (!(I2C->ISR & I2C_ISR_RXNE)) { }
    *Data++ = I2C->RXDR;
  }
}
//------------------------------------------------------------------------------

static void Write(I2C_TypeDef* I2C, unsigned char Address, const unsigned char* Data, unsigned char Size)
{
  I2C->CR2 &= ~(I2C_CR2_SADD_Msk | I2C_CR2_RD_WRN | I2C_CR2_NBYTES_Msk);
  I2C->CR2 |= (Address << (I2C_CR2_SADD_Pos + 1)) | (((unsigned long)Size)<<I2C_CR2_NBYTES_Pos);
  I2C->CR2 |= I2C_CR2_START;

  while(I2C->CR2 & I2C_CR2_START) { asm volatile("NOP"); }
  
  while (Size--)
  {
    while (!(I2C->ISR & I2C_ISR_TXE)) { asm volatile("NOP"); }
    I2C->TXDR = *Data++;
  }
  
  while(!(I2C->ISR & I2C_ISR_TC))  { asm volatile("NOP"); }
}
//------------------------------------------------------------------------------

static bool Trans(I2C_Data& I2C, I2C_Request* Request, unsigned char Address, const unsigned char* Data, unsigned char SizeWrite, unsigned char SizeRead)
{
  if(Request->Address || SizeWrite>Request->Size || SizeRead>Request->Size || (SizeWrite==0 && SizeRead==0)) return false;
  
  if(SizeWrite) memcpy(Request->Buffer, Data, SizeWrite);
  Request->Write=SizeWrite;
  Request->Read=SizeRead;
  
  Request->Address=Address;
  
  do
  {
    I2C_Request* dev = (I2C_Request*)__LDREXW((volatile unsigned int*)&I2C.Head);
    Request->Next=dev;
  }while(__STREXW((unsigned int)Request, (volatile unsigned int*)&I2C.Head));
  
  if(I2C.I2C->CR1 == I2C_CR1_PE) NVIC_SetPendingIRQ(I2C.IRQn);
  //---
  return true;
}
//------------------------------------------------------------------------------

static void Write2(I2C_TypeDef* I2C, unsigned char Address, const unsigned char* Data1, unsigned char Size1, const unsigned char* Data2, unsigned char Size2)
{
  I2C->CR2 &= ~(I2C_CR2_SADD_Msk | I2C_CR2_RD_WRN | I2C_CR2_NBYTES_Msk);
  I2C->CR2 |= (Address << (I2C_CR2_SADD_Pos + 1)) | (((unsigned long)Size1+Size2)<<I2C_CR2_NBYTES_Pos);
  I2C->CR2 |= I2C_CR2_START;
  
  while (Size1--)
  {
    while (!(I2C->ISR & I2C_ISR_TXE)) { asm volatile("NOP"); }
    I2C->TXDR = *Data1++;
  }
  
  while (Size2--)
  {
    while (!(I2C->ISR & I2C_ISR_TXE)) { asm volatile("NOP"); }
    I2C->TXDR = *Data2++;
  }
  
  while(!(I2C->ISR & I2C_ISR_TC))  { asm volatile("NOP"); }
}
//------------------------------------------------------------------------------

bool I2C2_Trans(I2C_Request* Request, unsigned char Address, const void* Data, unsigned char SizeWrite, unsigned char SizeRead)
{
  return Trans(I2C2_Data, Request, Address, (unsigned char*)Data, SizeWrite, SizeRead);
}

void I2C2_Write(unsigned char Address, unsigned char Data)
{
  Write(I2C2, Address, &Data, 1);
}
//------------------------------------------------------------------------------

void I2C2_Write(unsigned char Address, const void* Data, unsigned char Size)
{
  Write(I2C2, Address, (unsigned char*)Data, Size);
}
//------------------------------------------------------------------------------

void I2C2_Read(unsigned char Address, void* Data, unsigned char Size)
{
  Read(I2C2, Address, (unsigned char*)Data, Size);
}
//------------------------------------------------------------------------------

bool I2C2_CheckDevice(unsigned char Address)
{
  return I2C_CheckDevice(I2C2, Address);
}
//------------------------------------------------------------------------------
void I2C2_Stop()
{
  Stop(I2C2);
}
//------------------------------------------------------------------------------

bool I2C1_Trans(I2C_Request* Request, unsigned char Address, const void* Data, unsigned char SizeWrite, unsigned char SizeRead)
{
  return Trans(I2C1_Data, Request, Address, (unsigned char*)Data, SizeWrite, SizeRead);
}

void I2C1_Write(unsigned char Address, unsigned char Data)
{
  Write(I2C1, Address, &Data, 1);
}
//------------------------------------------------------------------------------

void I2C1_Write(unsigned char Address, const void* Data, unsigned char Size)
{
  Write(I2C1, Address, (unsigned char*)Data, Size);
}
//------------------------------------------------------------------------------

void I2C1_Write2(unsigned char Address, const void* Data1, unsigned char Size1, const void* Data2, unsigned char Size2)
{
  Write2(I2C1, Address, (unsigned char*)Data1, Size1, (unsigned char*)Data2, Size2);
}
//------------------------------------------------------------------------------

void I2C1_Read(unsigned char Address, void* Data, unsigned char Size)
{
  Read(I2C1, Address, (unsigned char*)Data, Size);
}
//------------------------------------------------------------------------------

bool I2C1_CheckDevice(unsigned char Address)
{
  return I2C_CheckDevice(I2C1, Address);
}
//------------------------------------------------------------------------------
void I2C1_Stop()
{
  Stop(I2C1);
}
//------------------------------------------------------------------------------

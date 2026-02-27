#include "stm32g4xx.h"

#include <string.h>

#include "gpio.h"

#include "uart.h"

static const unsigned long LPUART1_RecvSize=512+1;
static const unsigned long LPUART1_SendSize=512+1;

static const unsigned long UART1_RecvSize=512+1;
static const unsigned long UART1_SendSize=512+1;

static const unsigned long UART2_RecvSize=512+1;
static const unsigned long UART2_SendSize=512+1;

static const unsigned long UART3_RecvSize=512+1;
static const unsigned long UART3_SendSize=512+1;

struct UART_Data
{
  USART_TypeDef* UART;

  struct
  {
    unsigned long Size;
    unsigned char* Buffer;
    unsigned short Head;
    unsigned short Tail;
    bool Overflow;
  } Recv, Send;
};

static unsigned char UART3_BufferRecv[UART3_RecvSize], UART3_BufferSend[UART3_SendSize];
static UART_Data UART3_Data { USART3, {sizeof(UART3_BufferRecv), UART3_BufferRecv, 0, 0, false}, {sizeof(UART3_BufferSend), UART3_BufferSend, 0, 0, false} };

static unsigned char UART2_BufferRecv[UART2_RecvSize], UART2_BufferSend[UART2_SendSize];
static UART_Data UART2_Data { USART2, {sizeof(UART2_BufferRecv), UART2_BufferRecv, 0, 0, false}, {sizeof(UART2_BufferSend), UART2_BufferSend, 0, 0, false} };

static unsigned char UART1_BufferRecv[UART1_RecvSize], UART1_BufferSend[UART1_SendSize];
static UART_Data UART1_Data { USART1, {sizeof(UART1_BufferRecv), UART1_BufferRecv, 0, 0, false}, {sizeof(UART1_BufferSend), UART1_BufferSend, 0, 0, false} };

static unsigned char LPUART1_BufferRecv[LPUART1_RecvSize], LPUART1_BufferSend[LPUART1_SendSize];
static UART_Data LPUART1_Data { LPUART1, {sizeof(LPUART1_BufferRecv), LPUART1_BufferRecv, 0, 0, false}, {sizeof(LPUART1_BufferSend), LPUART1_BufferSend, 0, 0, false} };

static void IRQHandler(UART_Data& Uart)
{
  unsigned long isr = Uart.UART->ISR;
  if(isr & USART_ISR_RXNE)
  {
    Uart.Recv.Buffer[Uart.Recv.Head++] = Uart.UART->RDR;
    if(Uart.Recv.Head>=Uart.Recv.Size) Uart.Recv.Head=0;
    if(Uart.Recv.Head==Uart.Recv.Tail) Uart.Recv.Overflow=true;
  }
  if(isr & USART_ISR_TXE)
  {
    if(Uart.Send.Head != Uart.Send.Tail)
    {
      Uart.UART->TDR = Uart.Send.Buffer[Uart.Send.Tail++];
      if(Uart.Send.Tail>=Uart.Send.Size) Uart.Send.Tail=0;
    }
    else Uart.UART->CR1 &= ~USART_CR1_TXEIE;
  }
}
//------------------------------------------------------------------------------

extern "C" void USART3_IRQHandler()
{
  IRQHandler(UART3_Data);
}
//------------------------------------------------------------------------------

extern "C" void USART2_IRQHandler()
{
  IRQHandler(UART2_Data);
}
//------------------------------------------------------------------------------

extern "C" void USART1_IRQHandler()
{
  IRQHandler(UART1_Data);
}
//------------------------------------------------------------------------------

extern "C" void LPUART1_IRQHandler()
{
  IRQHandler(LPUART1_Data);
}
//------------------------------------------------------------------------------

static void Init(USART_TypeDef* USART, unsigned long Freq, unsigned long Baud)
{
  USART->CR1 = 0;

  USART->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

  USART->BRR = Freq / Baud;

  USART->CR3 |= USART_CR3_OVRDIS;
  USART->CR1 |= USART_CR1_UE;
}
//------------------------------------------------------------------------------

void UART3_Init(unsigned long Baud)
{
  if (RCC->APB1ENR1 & RCC_APB1ENR1_USART3EN) return;

  RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;

  GPIO_InitPin(GPIO_PIN_10 | GPIO_PORT_B | GPIO_ALTER | GPIO_AF7 | GPIO_OSPEED_HIGH);
  GPIO_InitPin(GPIO_PIN_11 | GPIO_PORT_B | GPIO_ALTER | GPIO_AF7 | GPIO_OSPEED_HIGH);

  Init(USART3, SystemCoreClock, Baud);

  NVIC_SetPriority(USART3_IRQn, 0);
  NVIC_EnableIRQ(USART3_IRQn);
}
//------------------------------------------------------------------------------

void UART2_Init(unsigned long Baud)
{
  if (RCC->APB1ENR1 & RCC_APB1ENR1_USART2EN) return;

  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

  GPIO_InitPin(GPIO_PIN_3 | GPIO_PORT_B | GPIO_ALTER | GPIO_AF7 | GPIO_OSPEED_HIGH);
  GPIO_InitPin(GPIO_PIN_4 | GPIO_PORT_B | GPIO_ALTER | GPIO_AF7 | GPIO_OSPEED_HIGH);

  Init(USART2, SystemCoreClock, Baud);

  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);
}
//------------------------------------------------------------------------------

void UART1_Init(unsigned long Baud)
{
  if (RCC->APB2ENR & RCC_APB2ENR_USART1EN) return;

  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  GPIO_InitPin(GPIO_PIN_6 | GPIO_PORT_B | GPIO_ALTER | GPIO_AF7 | GPIO_OSPEED_HIGH);
  GPIO_InitPin(GPIO_PIN_7 | GPIO_PORT_B | GPIO_ALTER | GPIO_AF7 | GPIO_OSPEED_HIGH);

  Init(USART1, SystemCoreClock, Baud);

  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);
}
//------------------------------------------------------------------------------

void LPUART1_Init(unsigned long Baud)
{
  if (RCC->APB1ENR2 & RCC_APB1ENR2_LPUART1EN) return;

  RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;

  GPIO_InitPin(GPIO_PIN_2 | GPIO_PORT_A | GPIO_ALTER | GPIO_AF12 | GPIO_OSPEED_HIGH);
  GPIO_InitPin(GPIO_PIN_3 | GPIO_PORT_A | GPIO_ALTER | GPIO_AF12 | GPIO_OSPEED_HIGH);
  
  Init(LPUART1, SystemCoreClock, Baud/256);

  NVIC_SetPriority(LPUART1_IRQn, 0);
  NVIC_EnableIRQ(LPUART1_IRQn);
}
//------------------------------------------------------------------------------

static unsigned long Recv(UART_Data& Uart, void* Data, unsigned long Size)
{
  if(!Data) 
  {
    if(Uart.Recv.Tail<=Uart.Recv.Head) return Uart.Recv.Head-Uart.Recv.Tail;
    return Uart.Recv.Size-Uart.Recv.Tail+Uart.Recv.Head;
  }
  
  unsigned char* data=(unsigned char*)Data;
  unsigned long size = 0;
  
  while (size < Size)
  {
    if (Uart.Recv.Head==Uart.Recv.Tail) break;
    data[size++]=Uart.Recv.Buffer[Uart.Recv.Tail++];
    if(Uart.Recv.Tail>=Uart.Recv.Size) Uart.Recv.Tail=0;
  }
  return size;
}
//------------------------------------------------------------------------------

static unsigned long Send(UART_Data& Uart, const void* Data, unsigned long Size)
{
  if(!Data) 
  {
    if(Uart.Send.Tail<=Uart.Send.Head) return Uart.Send.Head-Uart.Send.Tail;
    return Uart.Send.Size-Uart.Send.Tail+Uart.Send.Head;
  }
  
  unsigned char* data=(unsigned char*)Data;
  unsigned long size = 0;
  
  while (size < Size)
  {
    unsigned short head=Uart.Send.Head+1;
    if(head>=Uart.Send.Size) head=0;
    if (head == Uart.Send.Tail) { Uart.Send.Overflow=true; break; }
    Uart.Send.Buffer[Uart.Send.Head] = data[size++];
    Uart.Send.Head=head;
  }

  Uart.UART->CR1 |= USART_CR1_TXEIE;

  return size;
}
//------------------------------------------------------------------------------

static inline void Flush(UART_Data& Uart)
{
  Uart.Recv.Tail=Uart.Recv.Head=0;
}
//------------------------------------------------------------------------------

unsigned long UART3_Recv(void* Data, unsigned long Size)
{
  return Recv(UART3_Data, Data, Size);
}
//------------------------------------------------------------------------------

void UART3_Flush()
{
  Flush(UART3_Data);
}
//------------------------------------------------------------------------------

unsigned long UART3_Send(const void* Data, unsigned long Size)
{
  return Send(UART3_Data, Data, Size);
}
//------------------------------------------------------------------------------

unsigned long UART2_Recv(void* Data, unsigned long Size)
{
  return Recv(UART2_Data, Data, Size);
}
//------------------------------------------------------------------------------

void UART2_Flush()
{
  Flush(UART2_Data);
}
//------------------------------------------------------------------------------

unsigned long UART2_Send(const void* Data, unsigned long Size)
{
  return Send(UART2_Data, Data, Size);
}
//------------------------------------------------------------------------------

unsigned long UART1_Recv(void* Data, unsigned long Size)
{
  return Recv(UART1_Data, Data, Size);
}
//------------------------------------------------------------------------------

void UART1_Flush()
{
  Flush(UART1_Data);
}
//------------------------------------------------------------------------------

unsigned long UART1_Send(const void* Data, unsigned long Size)
{
  return Send(UART1_Data, Data, Size);
}
//------------------------------------------------------------------------------

unsigned long LPUART1_Recv(void* Data, unsigned long Size)
{
  return Recv(LPUART1_Data, Data, Size);
}
//------------------------------------------------------------------------------

void LPUART1_Flush()
{
  Flush(LPUART1_Data);
}
//------------------------------------------------------------------------------

unsigned long LPUART1_Send(const void* Data, unsigned long Size)
{
  return Send(LPUART1_Data, Data, Size);
}
//------------------------------------------------------------------------------

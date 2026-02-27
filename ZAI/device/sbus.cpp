#include "stm32g4xx.h"

#include <string.h>

#include "uart.h"
#include "tick.h"

#include "sbus.h"

#define JOY_MIN 240

#define JOY_VAL 1.27632418634f

#pragma pack(push,1)
union SBUS_Struct
{
  unsigned char data[25];
  struct
  {
    unsigned start : 8;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
    unsigned ch16 : 11;
    unsigned flags : 8;
    unsigned end : 8;
  } bus;
};
#pragma pack(pop)

// IBUS: LEN(1)+CMD(1)+DATA(0..32)+CHSM(2)

#define SBUS_START 0x0F

#define SBUS_CH17 0x01
#define SBUS_CH18 0x02

#define SBUS_FRAMELOST 0x04
#define SBUS_FAILSAFE 0x08

#define SBUS_LENGTH 25

void SBUS_Init()
{
  LPUART1_Init(100'000);
  LPUART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
  LPUART1->CR2 = USART_CR2_STOP_1 | USART_CR2_RXINV;
  LPUART1->CR1 |= USART_CR1_PCE | USART_CR1_PS | USART_CR1_UE;
}

static const unsigned long Size = SBUS_LENGTH;
static char Buffer[Size];
static bool Active = false;
static char Length = 0;

static unsigned long Time;

static bool Parse(SBUS_Data& Data, char byte)
{
  unsigned long tick=TICK_GetCount();
  
  unsigned long wait = tick - Time;
  if (wait > 4) 
    Length = 0; // Protocol synchronization lost !!!

  Time=tick;
  
  if (!Length && (byte != SBUS_START))
    return false;

  Buffer[Length++] = byte;
  
  if(Length<Size)
    return false;
  Length=0;
  
  SBUS_Struct* frame=(SBUS_Struct*)Buffer;
  if(frame->bus.end!=0)
    return false;
  
  bool fail = frame->bus.flags & SBUS_FAILSAFE;
  Data.FrameLost = frame->bus.flags & SBUS_FRAMELOST;
  if (Active)
  { 
    Data.FailSafe = fail; 
    if(Data.FailSafe) return true;
  }
  else if (fail) return true;
  
  Data.X=((float)(frame->bus.ch1-JOY_MIN))*JOY_VAL;
  Data.Y=((float)(frame->bus.ch2-JOY_MIN))*JOY_VAL;
  Data.Z=((float)(frame->bus.ch3-JOY_MIN))*JOY_VAL;
  Data.W=((float)(frame->bus.ch4-JOY_MIN))*JOY_VAL;
  
  Data.SWA=((float)(frame->bus.ch5-JOY_MIN))*JOY_VAL;
  Data.SWB=((float)(frame->bus.ch6-JOY_MIN))*JOY_VAL;
  Data.SWC=((float)(frame->bus.ch7-JOY_MIN))*JOY_VAL;
  Data.SWD=((float)(frame->bus.ch8-JOY_MIN))*JOY_VAL;
  
  Data.VRA=((float)(frame->bus.ch9-JOY_MIN))*JOY_VAL;
  Data.VRB=((float)(frame->bus.ch10-JOY_MIN))*JOY_VAL;
  
  Data.OTHER[0]=((float)(frame->bus.ch11-JOY_MIN))*JOY_VAL;
  Data.OTHER[1]=((float)(frame->bus.ch12-JOY_MIN))*JOY_VAL;
  Data.OTHER[2]=((float)(frame->bus.ch13-JOY_MIN))*JOY_VAL;
  Data.OTHER[3]=((float)(frame->bus.ch14-JOY_MIN))*JOY_VAL;
  Data.OTHER[4]=((float)(frame->bus.ch15-JOY_MIN))*JOY_VAL;
  Data.OTHER[5]=((float)(frame->bus.ch16-JOY_MIN))*JOY_VAL;
  
  Data.OTHER[6]=(bool)(frame->bus.flags & SBUS_CH17);
  Data.OTHER[7]=(bool)(frame->bus.flags & SBUS_CH18);
  Active = true;
  Data.Active = Active;
  return true;
}

bool SBUS_Update(SBUS_Data& Data)
{
  char buf[Size];
  unsigned long size = LPUART1_Recv(buf, Size);

  bool done = false;
  for (long a = 0; a < size; a++) done = Parse(Data, buf[a]);
  
  return done;
}
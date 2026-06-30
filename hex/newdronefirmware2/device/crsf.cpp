#include "stm32g4xx.h"

#include <string.h>

#include "uart.h"
#include "tick.h"

#include "crsf.h"

#define JOY_MIN 172

#define JOY_VAL 1.220256253f

#pragma pack(push,1)
union CRSF_Joypad
{
  unsigned char data[22];
  struct
  {
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
  } bus;
};
#pragma pack(pop)

struct CRSF_Head
{
  unsigned char Sync;
  unsigned char Length;
  unsigned char Type;
  unsigned char Data[];
};

// IBUS: LEN(1)+CMD(1)+DATA(0..32)+CHSM(2)

#define CRSF_START 0xC8

#define CRSF_FRAMETYPE_GENERIC_TELEMETRY 0x2D

#define CRSF_MIN_LENGTH 4
#define CRSF_MAX_LENGTH 64

const uint8_t crc8_dvbs2_table[256]
{
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,   0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,   0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,   0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,   0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,   0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,   0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,   0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,   0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,   0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,   0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,   0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,   0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,   0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,   0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,   0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,   0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

bool AllowTelemetrySend = false;

static unsigned char crsf_crc8(unsigned char *data, unsigned char len) 
{
  unsigned char crc = 0;
  for (unsigned char a = 0; a < len; a++) 
  {
    crc = crc8_dvbs2_table[crc ^ data[a]];
  }
  return crc;
}

void CRSF_Init()
{
  LPUART1_Init(421'875);
}

static const unsigned long Size = CRSF_MAX_LENGTH;
static unsigned char BufferIN[Size], BufferOUT[Size];
static bool Active = false;
static char Length = 0;

static unsigned long Time;

bool Parse(JOYPAD_Data& Data, void* TeleData, unsigned long* TeleSize, char byte)
{
  unsigned long tick=TICK_GetCount();
  
  unsigned long wait = tick - Time;
  if (wait > 1) 
    Length = 0; // Protocol synchronization lost !!!

  Time=tick;
  
  if (!Length && (byte != CRSF_START))
    return false;

  if(Length>=CRSF_MAX_LENGTH) 
  {
    Length = 0;
    return false;
  }
  
  BufferIN[Length++] = byte;
  
  if(Length<CRSF_MIN_LENGTH)
    return false;
  
  CRSF_Head* head = (CRSF_Head*)BufferIN;
  
  if(Length<head->Length+2)
    return false;
  
  unsigned char crc_in = BufferIN[Length-1];
  
  unsigned char crc_calc = crsf_crc8(&BufferIN[2], head->Length-1);
  
  Length=0;
  
  if(crc_in != crc_calc) return false;
  
  if(head->Type == 0x16) // Payload joypad
  {
    CRSF_Joypad* frame=(CRSF_Joypad*)head->Data;
    
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
    
    Active = true;
    Data.Active = Active;
    return true;
  }
  
  if(head->Type == CRSF_FRAMETYPE_GENERIC_TELEMETRY) // GENERIC_TELEMETRY
  {
    unsigned long len = head->Length-2;
    if(!TeleData || !TeleSize || *TeleSize<len) return true;
    memcpy(TeleData, head->Data, len);
    return true;
  }
}

bool CRSF_Update(JOYPAD_Data& Data, void* TeleData, unsigned long* TeleSize)
{
  char buf[Size];
  unsigned long size = LPUART1_Recv(buf, Size);

  bool done = false;
  for (long a = 0; a < size; a++) done = Parse(Data, TeleData, TeleSize, buf[a]);
  
  return done;
}

void* CRSF_GetPacket(void* Data, unsigned long *Size)
{
  CRSF_Head* str = (CRSF_Head*)BufferOUT; // CRSF_Head + CRC

  str->Sync = CRSF_START;
  str->Length = *Size + 2; // +Type+CRC
  str->Type = CRSF_FRAMETYPE_GENERIC_TELEMETRY;
  memcpy(str->Data, Data, *Size);
  str->Data[*Size] = crsf_crc8(&str->Type, *Size + 1); // CRC (Type + Data)
  *Size = *Size + 4;
  return str;
}

void CRSF_SendFGT(void* Data, unsigned long Size)
{
  CRSF_Head* str = (CRSF_Head*)BufferOUT; // CRSF_Head + CRC

  str->Sync = CRSF_START;
  str->Length = Size + 2; // +Type+CRC
  str->Type = CRSF_FRAMETYPE_GENERIC_TELEMETRY;
  memcpy(str->Data, Data, Size);
  str->Data[Size] = crsf_crc8(&str->Type, Size + 1); // CRC (Type + Data)
  
  LPUART1_Send(BufferOUT, Size + 4); // + Sync + Length + Type + CRC
}

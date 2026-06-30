#include "gpio.h"

#include "uart.h"
#include "tick.h"

#include "camflow.h"

#pragma pack(push,1)
struct MsgCamFlow
{
  unsigned char STX;
  unsigned char Len;
  float X;
  float Y;
  float YAW;
};

struct CamFlowSend
{
  unsigned char STX;
  float Range;
};

#pragma pack(pop)

enum class DATA_MODE : unsigned char
{
  Begin,
  Data
};


static bool Update = false;
static CamFlowData Data = {0.0, 0.0, 0.0};
static unsigned char Buffer[32];
static unsigned char STX = 170;
static DATA_MODE Mode = DATA_MODE::Begin;
static unsigned long Length, Index;
static unsigned char MaxLength = 14;
static bool INIT = false;

static bool ParseMessage(unsigned char byte)
{
  switch (Mode)
  {
    case DATA_MODE::Begin:
    {
      if (byte != STX) return false;

      Index = 0;
      Buffer[Index++] = byte;
      Length = sizeof(MsgCamFlow);
      Mode = DATA_MODE::Data;
      return false;
    }

    case DATA_MODE::Data:
    {
      Buffer[Index++] = byte;
      if (Index < Length) return false;

      MsgCamFlow& msg = *(MsgCamFlow*)Buffer;

      if (((unsigned short)msg.Len > MaxLength) || (Index < sizeof(MsgCamFlow)))
      {
        Mode = DATA_MODE::Begin;
        return false;
      }
      Data.X = msg.X;
      Data.Y = msg.Y;
      Data.YAW = msg.YAW;
      Update = true;
      Mode = DATA_MODE::Begin;
      return true;
    }
  }
}

bool CamFlow_Update()
{
  bool res;
  if (!INIT) return false;
  unsigned char buf[32];
  unsigned long size = UART2_Recv(&buf, sizeof(MsgCamFlow));
  if (size > 0)
  {
    for (unsigned long i = 0; i < size; i++) res = ParseMessage(buf[i]);
    return res;
  }
  return false;
}

void CamFlow_Init()
{
  UART2_Init(115200);
  INIT = true;
}

bool CamFlow_GetData(CamFlowData& data)
{ 
  if (Update)
  {
    data.X = Data.X;
    data.Y = Data.Y;
    data.YAW = Data.YAW;
    Update = false;
    return true;
  }
  return false;
}

void CamFlow_Send(float range)
{
  CamFlowSend msg={STX, range};
  UART2_Send(&msg, sizeof(CamFlowSend));
}

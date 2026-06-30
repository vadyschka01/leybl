#include <string.h>
#include <stdlib.h>

#include "gpio.h"

#include "uart.h"
#include "tick.h"

#include "tof.h"

#pragma pack(push,1)
struct HeadTOF
{
  unsigned short Header;     // 0x5959
  unsigned short Distance;
  unsigned short Strength;
  unsigned short Temprature; // Temprature/8 - 256;
  unsigned char  Check;      // Summ & 0xFF
};
#pragma pack(pop)

static const unsigned long TOF_Count=8;
static char TOF_Buffer[TOF_Count];
static unsigned long TOF_Size=0;

static float TOF_Range=0, TOF_Strength=0;

static const float TOF_Minimum=0.01f;
static const float TOF_Maximum=8.00f;

static const float TOF_Minimum2=0.10f;
static const float TOF_Maximum2=43.00f;

static bool TOF_Ready=false;

void TOF_Init()
{
  UART3_Init(115200);
}

void TOF_Update()
{
  char c;
  if(UART3_Recv(&c, 1))
  {
    if(c==' ')
    {
      TOF_Size=1;
      TOF_Buffer[0]=c;
      return;
    }
    
    if(!TOF_Size) return;

    if(c=='\r')
    {
      if(TOF_Size<4) 
      {
        TOF_Size=0;
        return;
      }
      
      TOF_Buffer[TOF_Size]='\0';
      
      char* end;
      float range=strtof(TOF_Buffer, &end);
      if(end-TOF_Buffer==TOF_Size) 
      {
        if(range>TOF_Minimum && range<TOF_Maximum) TOF_Range=range;
        else TOF_Range=0.0f;
        
        TOF_Ready=true;
      }
      else
        range=0;
      TOF_Size=0;
      return;
    }
    
    TOF_Buffer[TOF_Size++]=c;
    
    if(TOF_Size>=TOF_Count)
    {
      TOF_Size=0;
      return;
    }
    
  }
  
}

bool TOF_Update2()
{
  static HeadTOF buffer;
  static unsigned long index=0;
  char* data=(char*)&buffer;
  
  char c;
  while(UART3_Recv(&c, 1))
  {
    if(index<2)
    {
      if(c==0x59)
      {
        data[index++]=c;
        continue;
      }
      else 
      {
        index=0;
        break;
      }
    }
    
    data[index++]=c;
    
    if(index<sizeof(HeadTOF)) continue;
    
    index=0;
    
    unsigned char check=0;
    for(int a=0; a<sizeof(HeadTOF)-1; a++) check+=data[a];
    
    if(buffer.Check!=check) break;
    
    float range=((float)buffer.Distance)/100.0f;
    
    if(range<TOF_Minimum) TOF_Range=TOF_Minimum;
    else if(range>TOF_Maximum) TOF_Range=TOF_Maximum;
    else TOF_Range=range;
    
    TOF_Strength=buffer.Strength;
    
    TOF_Ready=true;
    
    return true;
  }
  
  return false;
}

bool TOF_Update3()
{
  static HeadTOF buffer;
  static unsigned long index=0;
  char* data=(char*)&buffer;
  
  char c;
  while(UART3_Recv(&c, 1))
  {
    if(index<2)
    {
      if(c==0x59)
      {
        data[index++]=c;
        continue;
      }
      else 
      {
        index=0;
        break;
      }
    }
    
    data[index++]=c;
    
    if(index<sizeof(HeadTOF)) continue;
    
    index=0;
    
    unsigned char check=0;
    for(int a=0; a<sizeof(HeadTOF)-1; a++) check+=data[a];
    
    if(buffer.Check!=check) break;
    
    //float range=((float)buffer.Distance)/100.0f;
    float range = (data[2] | (data[3] << 8)) / 100.0f;
    
    if(range<TOF_Minimum2) TOF_Range=TOF_Minimum2;
    else if(range>TOF_Maximum2) TOF_Range=TOF_Maximum2;
    else TOF_Range=range;
    
    //TOF_Strength=buffer.Strength;
    TOF_Strength = data[4] | (data[5] << 8);
    
    TOF_Ready=true;
    
    return true;
  }
  
  return false;
}

bool TOF_GetRange(float& Range, float& Strength)
{
  bool ready=TOF_Ready;
  TOF_Ready=false;
  
  Range=TOF_Range;
  Strength=TOF_Strength;
  
  return ready;
}

#include "stm32g4xx.h"

#include <string.h>

#include "flash.h"

#define FLASH_FKEY1 0x45670123
#define FLASH_FKEY2 0xCDEF89AB

void FLASH_Erase(unsigned long Page)
{
  __disable_irq();
  
  while(FLASH->SR & FLASH_SR_BSY) { }
  //---
  if(FLASH->CR & FLASH_CR_LOCK) 
  { 
    FLASH->KEYR = FLASH_FKEY1;
    FLASH->KEYR = FLASH_FKEY2;
  }
  //---
  while (FLASH->SR & FLASH_SR_BSY) { }
  //---
  FLASH->SR = FLASH_SR_OPTVERR | FLASH_SR_RDERR | FLASH_SR_FASTERR | FLASH_SR_MISERR | FLASH_SR_SIZERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_PROGERR | FLASH_SR_OPERR | FLASH_SR_EOP;
  
  FLASH->CR &= ~FLASH_CR_PNB_Msk;
  
  FLASH->CR |= FLASH_CR_PER | (Page<<FLASH_CR_PNB_Pos);
  FLASH->CR |= FLASH_CR_STRT;
  
  while (FLASH->SR & FLASH_SR_BSY) { }
  
  FLASH->SR = FLASH_SR_EOP;
  FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PNB_Msk);
  //---
  FLASH->CR |= FLASH_CR_LOCK;
  
  __enable_irq();
}
//-------------------------------------------------------------

void FLASH_Write(unsigned long I_Addr, const void* I_Data, unsigned long I_Size)
{
  __disable_irq();
  
  while(FLASH->SR & FLASH_SR_BSY) { }
  //---
  if(FLASH->CR & FLASH_CR_LOCK) 
  { 
    FLASH->KEYR = FLASH_FKEY1;
    FLASH->KEYR = FLASH_FKEY2;
  }
  //---
  unsigned long long* read=(unsigned long long*)I_Data;
  volatile unsigned long long* write=(volatile unsigned long long*)I_Addr;
  //---
  
  unsigned long size=I_Size/8;
  for(unsigned long a=0; a<size; a++)
  {
    while(FLASH->SR & FLASH_SR_BSY) {  }
    
    FLASH->SR = FLASH_SR_OPTVERR | FLASH_SR_RDERR | FLASH_SR_FASTERR | 
                FLASH_SR_MISERR | FLASH_SR_SIZERR | FLASH_SR_PGAERR | 
                FLASH_SR_WRPERR | FLASH_SR_PROGERR | FLASH_SR_OPERR | FLASH_SR_EOP;
    //---
    FLASH->CR |= FLASH_CR_PG;
    
    write[a]=read[a];
    
    while(FLASH->SR & FLASH_SR_BSY) {  }
    
    FLASH->CR &= ~FLASH_CR_PG;
  }
  //---
  
  if(I_Size & 7)
  {
    unsigned char* data=((unsigned char*)I_Data)+size*8;
    
    unsigned long last = I_Size & 7;
    
    unsigned long long read=0xFFFFFFFFFFFFFFFFULL;
    memcpy(&read, data, last);
      
    while(FLASH->SR & FLASH_SR_BSY) {  }
    
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_WRPERR;
    //---
    FLASH->CR |= FLASH_CR_PG;
    
    write[size]=read;
    
    while(FLASH->SR & FLASH_SR_BSY) {  }
    
    FLASH->CR &= ~FLASH_CR_PG;
  }
  
  __enable_irq();
  
  //---
  FLASH->CR |= FLASH_CR_LOCK;
}
//-------------------------------------------------------------

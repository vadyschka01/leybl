#pragma once

#define FLASH_ADDR_MASK   0xFFFFFFFEUL

#define FLASH_INFO_START 0x08002000UL
#define FLASH_INFO_END   0x08003000UL

#define FLASH_INFO_PAGE  2

#define FLASH_NORM_START 0x08013000UL
#define FLASH_NORM_END   0x08014000UL

#define FLASH_NORM_PAGE  19

void FLASH_Erase(unsigned long Page);

void FLASH_Write(unsigned long I_Addr, const void* I_Data, unsigned long I_Size);

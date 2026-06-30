#pragma once

#include "joy.h"

void CRSF_Init();
bool Parse(JOYPAD_Data& Data, void* TeleData, unsigned long* TeleSize, char byte);
bool CRSF_Update(JOYPAD_Data& Data, void* TeleData, unsigned long* TeleSize);
void* CRSF_GetPacket(void* Data, unsigned long *Size);
void CRSF_SendFGT(void* Data, unsigned long Size);
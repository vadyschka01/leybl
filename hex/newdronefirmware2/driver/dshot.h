#include "stm32g4xx.h"

bool DSHOT_Init(unsigned long Freq, unsigned long Timer, bool Bidirect);

bool DSHOT_Mode(unsigned long Freq, unsigned long Timer, bool Bidirect);

void DSHOT_SetCommand(unsigned short Command[4]); // tele - 0x8000 | data - 0x0FFF

void DSHOT_GetERPM(unsigned short eRPM[4], bool Error[4]);

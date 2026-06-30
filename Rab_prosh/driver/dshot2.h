#include "stm32g4xx.h"

bool DSHOT_Init(unsigned long Freq, bool Bidirect);

bool DSHOT_Mode(unsigned long Freq, bool Bidirect);

bool DSHOT_SetCommand(unsigned short Command[4], bool Forced); // tele - 0x8000 | data - 0x0FFF

bool DSHOT_GetERPM(unsigned short eRPM[4], bool Error[4]);

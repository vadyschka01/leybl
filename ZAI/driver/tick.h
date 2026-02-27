#pragma once

void TICK_Init();
unsigned long TICK_GetCount();
float TICK_GetTime();
void TICK_Delay(unsigned long us); // 900 us maximum

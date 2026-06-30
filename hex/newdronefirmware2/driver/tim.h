#pragma once

typedef void (*ProcTIM)();

void TIM7_Init(long Priority, unsigned long Freq, const ProcTIM& Proc1, const ProcTIM& Proc2, const ProcTIM& Proc3);

void TIM7_Enable();

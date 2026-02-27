#pragma once

typedef void (*ProcTIM)();

void TIM6_Init(long Priority, unsigned long Freq, const ProcTIM& Proc1, const ProcTIM& Proc2);
void TIM7_Init(long Priority, unsigned long Freq, const ProcTIM& Proc);

void TIM6_Enable();
void TIM7_Enable();

#pragma once

extern bool PWM_Enable;

void PWM_Init(unsigned long Freq);
void PWM_SetQuad(short M[4], unsigned short Min, unsigned short Max);
void PWM_SetHexa(short M[6], unsigned short Min, unsigned short Max);
void PWM_SetAll(unsigned short Pow);

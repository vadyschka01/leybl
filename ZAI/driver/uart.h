#pragma once

void UART3_Init(unsigned long Baud);
void UART3_Flush();
unsigned long UART3_Recv(void* Data, unsigned long Size);
unsigned long UART3_Send(const void* Data, unsigned long Size);

void UART2_Init(unsigned long Baud);
void UART2_Flush();
unsigned long UART2_Recv(void* Data, unsigned long Size);
unsigned long UART2_Send(const void* Data, unsigned long Size);

void UART1_Init(unsigned long Baud);
void UART1_Flush();
unsigned long UART1_Recv(void* Data, unsigned long Size);
unsigned long UART1_Send(const void* Data, unsigned long Size);

void LPUART1_Init(unsigned long Baud);
void LPUART1_Flush();
unsigned long LPUART1_Recv(void* Data, unsigned long Size);
unsigned long LPUART1_Send(const void* Data, unsigned long Size);
#pragma once

void SPI1_Init();

void SPI1_Transfer(const void* WriteData, void* ReadData, unsigned long Size);

void SPI1_CS_OF(bool En);

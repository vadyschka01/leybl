#ifndef SBUS_H
#define SBUS_H

#include <stdint.h>

// === Глобальные переменные SBUS ===
extern volatile uint16_t rc_channels[16];
extern volatile uint8_t sbus_failsafe;
extern volatile uint8_t sbus_frame_lost;

extern volatile uint32_t sbus_packets_ok;
extern volatile uint32_t sbus_decode_errors;

extern volatile uint8_t  rx_buffer[25];
extern volatile uint8_t  rx_idx;
extern volatile uint8_t  parser_state;

extern volatile uint32_t sbus_last_byte_time;
extern volatile uint32_t sbus_hz;
extern volatile uint32_t sbus_hz_counter;

// === Функции ===
void LPUART1_SBUS_Init(void);
void SBUS_Decode(void);

#endif

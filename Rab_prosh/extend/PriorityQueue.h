#pragma once

#define MAX_HEAP_SIZE 10        // Максимальное количество сообщений в куче
#define MAX_MSG_PAYLOAD 100     // Максимальный размер одного сообщения

struct PriorityMessage
{
  unsigned char data[MAX_MSG_PAYLOAD];
  unsigned long size;
  int priority;
};

bool PriorityQueue_Push(const void* data, unsigned long size, int priority);
bool PriorityQueue_Pop(PriorityMessage* outMsg);
#pragma once

#include "PriorityQueue.h"

static PriorityMessage heap[MAX_HEAP_SIZE];
static int heapSize = 0;


static void swapMessages(int i, int j) 
{
    PriorityMessage temp = heap[i];
    heap[i] = heap[j];
    heap[j] = temp;
}

static void siftUp(int idx) 
{
  while (idx > 0) 
  {
    int parent = (idx - 1) / 2;
    if (heap[idx].priority <= heap[parent].priority) break;
    swapMessages(idx, parent);
    idx = parent;
  }
}

static void siftDown(int idx) 
{
  while (true) 
  {
    int left = 2 * idx + 1;
    int right = 2 * idx + 2;
    int largest = idx;

    if (left < heapSize && heap[left].priority > heap[largest].priority) {
        largest = left;
    }
    if (right < heapSize && heap[right].priority > heap[largest].priority) {
        largest = right;
    }
    if (largest == idx) break;

    swapMessages(idx, largest);
    idx = largest;
  }
}

bool PriorityQueue_Push(const void* data, unsigned long size, int priority) 
{
    if (heapSize >= MAX_HEAP_SIZE || size > MAX_MSG_PAYLOAD) return false;

    // Вставляем в конец
    heap[heapSize].size = size;
    heap[heapSize].priority = priority;
    
    const unsigned char* pSrc = (const unsigned char*)data;
    for (unsigned long i = 0; i < size; i++) 
    {
      heap[heapSize].data[i] = pSrc[i];
    }

    // Восстанавливаем свойства кучи
    siftUp(heapSize);
    heapSize++;
    return true;
}

bool PriorityQueue_Pop(PriorityMessage* outMsg) 
{
    if (heapSize <= 0) return false;

    // Корень (индекс 0) — всегда самый приоритетный
    *outMsg = heap[0];

    // Ставим последний элемент на место корня и опускаем его
    heapSize--;
    if (heapSize > 0) 
    {
      heap[0] = heap[heapSize];
      siftDown(0);
    }
    return true;
}
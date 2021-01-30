#ifndef CIRCULAR_BUFFER_H__
#define CIRCULAR_BUFFER_H__

#include <stdbool.h>
#include <stdint.h>

#define CIRCULAR_BUFFER_LENGTH 50

typedef struct circular_buffer_t {
  uint16_t data[CIRCULAR_BUFFER_LENGTH];
  uint16_t head;
  uint16_t tail;
} circular_buffer_t;

void push_back(circular_buffer_t* cb, uint16_t val);
void reset(circular_buffer_t* cb, uint16_t val);
uint16_t front(circular_buffer_t* cb);
uint16_t back(circular_buffer_t* cb);
uint16_t length(circular_buffer_t* cb);

#endif

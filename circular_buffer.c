#include "circular_buffer.h"

uint16_t length(circular_buffer_t* cb) {
  return sizeof(cb->data) / sizeof(cb->data[0]);
}

void push_back(circular_buffer_t* cb, uint16_t val) {
  cb->data[cb->tail] = val;
  cb->head = cb->tail;
  cb->tail = (cb->tail + 1) % length(cb);
}

void reset(circular_buffer_t* cb, uint16_t val) {
  cb->tail = 1;
  cb->head = 0;
  uint16_t N = length(cb);
  for(uint16_t i = 0; i < N; i++) {
    push_back(cb, val);
  }
}

uint16_t front(circular_buffer_t* cb) {
  return cb->data[cb->head];
}

uint16_t back(circular_buffer_t* cb) {
  return cb->data[cb->tail];
}


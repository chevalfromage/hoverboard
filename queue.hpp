#pragma once
#include <stdint.h>

template <typename VALUE_TYPE, unsigned int N_QUEUE>
struct Queue_t {
  volatile uint32_t head = 0;
  volatile uint32_t tail = 0;
  static_assert(
    (N_QUEUE && ((N_QUEUE & (N_QUEUE - 1)) == 0)),
    "queue size have to be a 2^N."
  );
  volatile VALUE_TYPE queue[N_QUEUE];

  inline bool is_empty(){return head == tail;}
  inline bool is_full(){return ((head+1)&(N_QUEUE -1)) == tail;}
  inline bool append(VALUE_TYPE const e){
    if( is_full() ) return true;
    queue[head] = e;
    head = ((head+1)&(N_QUEUE -1));
    return false;
  }
  inline VALUE_TYPE pop(){
    VALUE_TYPE tail_value = queue[tail];
    tail = ((tail+1)&(N_QUEUE -1));
    return tail_value;
  }
  inline void clear(){tail = head;}
  inline uint32_t size(){return (head - tail) & (N_QUEUE-1);}
};

#pragma once
#include "queue.hpp"

template <typename NUMERIC_TYPE, unsigned int ORDER>
struct RII_filter {
  static constexpr const int order = ORDER;
  static constexpr const int N = order +1;
  const NUMERIC_TYPE bb[N];
  const NUMERIC_TYPE aa[N];

  int pos;
  NUMERIC_TYPE signal[N];
  NUMERIC_TYPE filtered[N];
  inline void reset(NUMERIC_TYPE value = 0.0){
    for(int i=0; i<N; i++){
      signal[i] = value;
      filtered[i] = value;
    }
    pos = N-1;
  }
  inline void append(NUMERIC_TYPE value){
    pos--;
    if(pos<0){ pos=N-1; }
    signal[pos] = value;
    filtered[pos] = bb[0] * signal[pos];
    int cpt = pos;
    for(int i=1; i<N; i++){
      cpt++;
      if(cpt>=N){ cpt=0; } 
      filtered[pos] += (bb[i] * signal[cpt] - aa[i] * filtered[cpt]);
    }
  }
  inline NUMERIC_TYPE get_original_value(){
    return signal[pos];
  }
  inline NUMERIC_TYPE get_value(){
    return filtered[pos];
  }
};


template <typename NUMERIC_TYPE, unsigned int QUEUE_SIZE>
struct Decimation_queue {
  unsigned int N_decimation;
  unsigned int cpt_decimation = 0;
  Queue_t<NUMERIC_TYPE, QUEUE_SIZE> decimed_signal;

  inline void append(NUMERIC_TYPE value){
    cpt_decimation++;
    if(cpt_decimation == N_decimation ){
      cpt_decimation = 0;
    }
    if( cpt_decimation == 0 ){
      decimed_signal.append(value);
    }
  }

  Decimation_queue(unsigned int N_decimation):
    N_decimation(N_decimation)
  { }

  inline void reset(){
    cpt_decimation = 0;
    decimed_signal.clear();
  }

  inline bool get_avalaible_value(NUMERIC_TYPE& value, bool& loss_data){
    bool result = false;
    if( (result = !decimed_signal.is_empty()) ){
      value = decimed_signal.pop();
      loss_data = false;
      while(!decimed_signal.is_empty()){
        decimed_signal.pop();
        loss_data = true;
      }
    }
    return result;
  }
};


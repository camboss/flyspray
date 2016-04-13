#ifndef __CB_H_INCLUDED__   // if x.h hasn't been included yet...
#define __CB_H_INCLUDED__

#include <array>
#include "Arduino.h"

class buffer{
  std::array<uint32_t,3> measurements;
  int index = 0;
  int count = 0;
 
  public:
  void push(uint32_t);
  uint32_t getAverage(void); 
  void clearBuffer(void);
  bool averageReady();
};

#endif

#include "buffer.h"

void buffer::push(uint32_t value)
{
  measurements[index] = value;
  if (index++ >= measurements.size()){
    index = 0;
  }
  if (count++ > measurements.size()){
    count = measurements.size(); 
  }
}

bool buffer::averageReady(void)
{
  if (count >= measurements.size()){
    return true;
  }
  return false;
}

uint32_t buffer::getAverage(void)
{
  uint32_t sum = 0;
  for (int i=0; i<measurements.size(); i++) {
    sum += measurements[i];
  }
  return (uint32_t) sum / measurements.size();
}

void buffer::clearBuffer(void)
{
  index = 0;
  count = 0;
}

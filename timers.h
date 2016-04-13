#ifndef timers_h
#define timers_h

#include "Arduino.h"

#define FREQ_SLOW_CLOCK_EXT 32768

void TimerInit(Tc *tc, uint32_t channel, IRQn_Type irq, uint8_t clock, uint32_t ms);
void TimerStart(Tc *tc, uint32_t channel, uint8_t clock, uint32_t ms);

#endif

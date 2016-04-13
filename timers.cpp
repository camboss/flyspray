#include "timers.h"

void TimerInit(Tc *tc, uint32_t channel, IRQn_Type irq, uint8_t clock, uint32_t ms)
{
   uint32_t rc;
   pmc_set_writeprotect(false);
   pmc_enable_periph_clk(irq);
   TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | clock);
   switch(clock){
     case TC_CMR_TCCLKS_TIMER_CLOCK1:
       rc = (int)((VARIANT_MCK/2)*ms)/1000;
     break;
     case TC_CMR_TCCLKS_TIMER_CLOCK2:
       rc = (int)((VARIANT_MCK/8)*ms)/1000;
     break;
     case TC_CMR_TCCLKS_TIMER_CLOCK3:
       rc = (int)((VARIANT_MCK/32)*ms)/1000;
     break;
     case TC_CMR_TCCLKS_TIMER_CLOCK4:
       rc = (int)((VARIANT_MCK/128)*ms)/1000;
     break;
     case TC_CMR_TCCLKS_TIMER_CLOCK5:
       rc = (int)(FREQ_SLOW_CLOCK_EXT*ms)/1000;
     break;
   }
   
   TC_SetRC(tc, channel, rc);
   TC_Start(tc, channel);
   tc->TC_CHANNEL[channel].TC_IER=  TC_IER_CPCS;
   tc->TC_CHANNEL[channel].TC_IDR= ~TC_IER_CPCS;
   NVIC_EnableIRQ(irq);
}

void TimerStart(Tc *tc, uint32_t channel, uint8_t clock, uint32_t ms){
  uint32_t rc;
  switch(clock){
    case TC_CMR_TCCLKS_TIMER_CLOCK1:
      rc = (int)((VARIANT_MCK/2)*ms)/1000;
      break;
     case TC_CMR_TCCLKS_TIMER_CLOCK2:
      rc = (int)((VARIANT_MCK/8)*ms)/1000;
      break;
    case TC_CMR_TCCLKS_TIMER_CLOCK3:
      rc = (int)((VARIANT_MCK/32)*ms)/1000;
      break;
    case TC_CMR_TCCLKS_TIMER_CLOCK4:
      rc = (int)((VARIANT_MCK/128)*ms)/1000;
      break;
    case TC_CMR_TCCLKS_TIMER_CLOCK5:
      rc = (int)(FREQ_SLOW_CLOCK_EXT*ms)/1000;
      break;
    }
  TC_SetRC(tc, channel, rc);
  TC_Start(tc,channel);   
}

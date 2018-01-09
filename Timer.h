/**********************************************************************************************************************************
Project     :  L7-Mirror
Filename    :  Timer.h
Brief descr.:  Headerfile for timer functionality (delays with us, millisecond counter)
Copyright   :  
Remarks     :  
Version
  1.0.0  18.09.2009  UF  Creation
**********************************************************************************************************************************/
#ifndef _TIMER_H_
#define _TIMER_H_

#include "params.h"
/* Include-Files ################################################################################################################*/
/* Defines ######################################################################################################################*/
#define TIMER_CYLE_MS (TIMER_CYLE_US/1000)
#define t_in_ms(ms_time) (((uint32)ms_time + TIMER_CYLE_MS) / TIMER_CYLE_MS)
#define t_in_us(us_time) (((uint32)us_time + TIMER_CYLE_US) / TIMER_CYLE_US)

/* Type-definitions #############################################################################################################*/
typedef uint16 typeTimerValue;
/* Declarations #################################################################################################################*/
#ifdef _TIMER_C_
# define extern
#endif
/* Global variables #############################################################################################################*/
/* Interface-functions ##########################################################################################################*/
extern void TimerInit( void);

// Slower timer (TIMER_CYLE_MS per tick)
extern typeTimerValue TimerGet( void);
extern typeTimerValue TimerGetDiffTicks( typeTimerValue start_ticks);

// microsecond timer
#define TimerGetUs() (TIM1->CNTRH*256+TIM1->CNTRL) // Returns the actual 16bit-value of the microsecond timer
extern uint16 TimerGetElapsedUs( uint16 t_start);
extern void TimerDelayUs( uint16 t_us);

#ifdef extern
# undef extern
#endif
#endif // _xxx_H_
/*-- End of File --*/

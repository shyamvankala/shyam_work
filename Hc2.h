#ifndef _HC2_H_
#define _HC2_H_


/**********************************************************************************************************************************
Project     :  L7-Mirror
Filename    :  Hc2.h
Brief descr.:  Headerfile for HC2 (Heading control) functions (control and diagnostics)
Copyright   :  
Remarks     :  
Version
  1.0.0  24.07.2017  shym  Creation
**********************************************************************************************************************************/
 #include "stdint.h"

#include "ZMDCM0.h"

//* Include-Files ################################################################################################################*/
/* Defines ######################################################################################################################*/
#define HC2_STATE_OFF   0 // no activity
#define HC2_STATE_ON    1 // constant brightness
#define HC2_STATE_FLASH 2 // flashing


#define HC2_UBAT_OUT(on)  		HS_vHSControl(eHS2,on)
#define PwmOut(period,duty )    PWM_vSettings(period,duty)


typedef union{
  uint8_t byte;
  struct {
    uint8 state :2; // Mainstate
    uint8 sub   :3; // Substate
    uint8 cnt   :3; // Counter
  }s;
}typeHc2State;

/* Global variables #############################################################################################################*/
extern typeHc2State Hc2State;

# define Hc2IsActive() (Hc2GetState().s.state!=HC2_STATE_OFF)
#define Hc2GetState() Hc2State

/* Declarations #################################################################################################################*/
#ifdef _HC2_C_
# define extern
#endif
/* Global variables #############################################################################################################*/
 //typeHc2State Hc2State;

/* Interface-functions ##########################################################################################################*/
extern void Hc2Init( void);
extern void Hc2Cyclic( void);

/* Interface-functions ##########################################################################################################*/
extern void Hc2Init( void);
#endif


 /**********************************************************************************************************************************
Project     :  INTLIN -Mirror
Filename    :  Htr.h
Brief descr.:  Headerfile for Htr (mirror glass heater) functions (control and diagnostics)
Copyright   :  
Remarks     :  

**********************************************************************************************************************************/
 
#ifndef __htr_h
#define __htr_h

 
 #include "stdint.h"

#include "ZMDCM0.h"

#include "SPI.h"

#include "SBC.h"

#include "SBC_cfg.h"

#include "HS_cfg.h"

#include "HS.h"
#include "LINCom.h"
#include "SBC_cfg.h"
 #include "LINCom_cfg.h"
 #include "Applinit.h"
#include "Vmon.h"

 #include "TMR32_Cfg.h"
#include "TMR32.h" 

#define HtrGetState() HtrState
#define HtrIsActive() (HtrGetState()==HTR_STATE_ON_ACTIVE)

 /* Type-definitions #############################################################################################################*/
#define HTR_STATE_IDLE       0
#define HTR_STATE_ON_ACTIVE  1
#define HTR_STATE_ON_PASSIVE 2


  // Parameters for Heater
 #define R_CM               8200.
 
#define HTR_CURR_MAX_AMPS     2.000   // maximum current
//#define HTR_CURR_MAX_TIME_MS    3000   // detection time for maximum current measured (or MCU-Flag immediately), integrated over several PWM-Periods
//#define HTR_CURR_MIN_TIME_MS    3000   // detection time for open load flag
#define HTR_NOMINAL_POWER_W       18.  // Nominal power in watts for 100% mirror heater (18 or 30)
//#define HTR_PWM_PERIOD_MS       100   // 1 Hz PWM frequency for heater
//#define HTR_CYCLE_TIME_MS       10
 #define HTR_I_CM_MA_FAKTOR       1.8

#define CR_LOW_RDS_ON  10000.
#define CR_HIGH_RDS_ON  2000.
#define HTR_I_CM_MA_PER_LSB  106//((const float)((VREF_NOM_MV*CR_LOW_RDS_ON)/(R_CM*ADC_MAX_LSBS)))
//#define I_CM_MEAS_MA_PER_LSB_HIGH_RDS_ON  ((const float)((VREF_NOM_MV*CR_HIGH_RDS_ON)/(R_CM*ADC_MAX_LSBS)))

#define HTR_OUT(on)        HS_vHSControl(eHS1,on);
//#define HTR_OUT()        HS_vHSControl(eHS1,eHS_ON);
/* Global variables #############################################################################################################*/
extern uint8_t HtrState;
 /* Interface-functions ##########################################################################################################*/
extern void HtrInit( void);
extern void HtrCyclic( void);

#endif

                                                                                         
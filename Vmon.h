/**********************************************************************************************************************************
Project     :  L7-Mirror
Filename    :  Vmon.h
Brief descr.:  Headerfile for voltage monitoring of supply voltage (+Ub)
Copyright   :  
Remarks     :  Other +Ub related voltages are converted synchronous to Ubat
Version
  1.0.0  18.09.2009  UF  Creation
**********************************************************************************************************************************/
#ifndef _VMON_H_
#define _VMON_H_
//#include "Hardware.h"
#include "ADC.h"
#include "stdint.h"
#include "debounce.h"
/* Include-Files ################################################################################################################*/
/* Defines ######################################################################################################################*/

#define VREF_NOM_MV     2.4 // Voltage at Vref nominal in millivolts
#define ADC_MAX_LSBS    1023
#define ADC_MAX_LSBS8    255
#define UBAT_MEAS_RHS        82.  //for calculation 
#define UBAT_MEAS_RLS        27. // for calculation
// Voltage divider for Ubat-monitoring
#define UBAT_MEAS_DIVIDER    (UBAT_MEAS_RLS/(UBAT_MEAS_RLS+UBAT_MEAS_RHS))

   #define UBAT_MEAS_MV_PER_LSB  ((const float) (VREF_NOM_MV)*8)

   #define UBAT_MEAS_VOLTS_TO_LSBS(val)  (((val)*ADC_MAX_LSBS)/ UBAT_MEAS_MV_PER_LSB)
   #define Ubat_Rescode 27 
   #define VOLTAGE_OFFSET(val)  ((ADC_MAX_LSBS - val)/Ubat_Rescode)
/* Type-definitions #############################################################################################################*/
/* Declarations #################################################################################################################*/
//#ifdef _VMON_C_
//# define extern
//#endif
/* Global variables #############################################################################################################*/
 //static volatile tADCMeasuredType UbatRaw ,UHc2Gnd;
 extern uint16_t UbatRaw ,UHc2Gnd;
 extern uint8_t bVmonFailure, bVmonFailureHtr;
/* Interface-functions ##########################################################################################################*/
extern void VmonInit( void);
extern void VmonCyclic( void);
extern uint8_t bVTFailure;

#define VmonGetFailure()    bVmonFailure
#define VmonGetFailureHtr() bVmonFailureHtr
#define VTFailure() bVTFailure 

#define VmonGetUbatRaw() UbatRaw
#define VmonGetUHc2Gnd() UHc2Gnd
#define VmonGetUHbridgeCom() UHbridgeCom


#if _DEBUG==ON // for debugging only  
extern const typeDebounceState* VmonGetDebounceStates( void);
#endif

#ifdef extern
# undef extern
#endif
#endif // _VMON_H_
/*-- End of File --*/

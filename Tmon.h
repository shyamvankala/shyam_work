/**********************************************************************************************************************************
Project     :  L7-Mirror
Filename    :  Tmon.h
Brief descr.:  Sourcefile for temperature monitoring
Copyright   :  
Remarks     :  
Version
  1.0.0  18.09.2009  UF  Creation
**********************************************************************************************************************************/
#ifndef _TMON_H_
#define _TMON_H_

/* Include-Files ################################################################################################################*/
  #include "STD_include.h"
/* Defines ######################################################################################################################*/
/* Type-definitions #############################################################################################################*/
#define TMON_OK    0
#define TMON_ERROR 1
/* Declarations #################################################################################################################*/
#ifdef _TMON_C_
# define extern
#endif
/* Global variables #############################################################################################################*/
extern uint16_t TmonTempRaw;
extern boolean TmonState;
/* Interface-functions ##########################################################################################################*/
extern void TmonInit( void);
extern void TmonCyclic( void);

#define TmonGetFailure()        (TmonState==TMON_ERROR)
#define TmonGetTemperatureRaw() TmonTempRaw
#define TmonGetState()          TmonState

#ifdef extern
# undef extern
#endif
#endif // _xxx_H_
/*-- End of File --*/

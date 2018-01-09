/**********************************************************************************************************************************
Project     :  L7-Mirror
Filename    :  Debounce.h
Brief descr.:  Headerfile for debounce functionality
Copyright   :  
Remarks     :  
Version
  1.0.0  17.10.2009  UF  Creation
**********************************************************************************************************************************/
#ifndef _DEBOUNCE_H_
#define _DEBOUNCE_H_

#include "stdint.h"
#include "std_init.h"

typedef unsigned char         boolean;
typedef unsigned char         uint8;         /*           0 .. 255             */
/* Include-Files ################################################################################################################*/
/* Defines ######################################################################################################################*/
#define DEBOUNCE_ACTION_NO        0
#define DEBOUNCE_ACTION_SUB_ERROR 1
#define DEBOUNCE_ACTION_LIN_ERROR 2

/* Type-definitions #############################################################################################################*/
typedef struct{
  uint8 State            :2;
  uint8 StableResultDet  :1;  // FALSE after initialization, TRUE after a stable result was detected and the action performed
  uint8 Reserved         :5;  
}typeDebounceState;

typedef uint16_t typeDebounceCnt;

typedef struct {
  typeDebounceState* pState;                // Pointer to state variable
  typeDebounceCnt*   pCounter;              // Pointer to counter (size depends on DEBOUNCE_MODE_xxx)
  typeDebounceCnt    SetDelay;              // Delay for setting (number of calls to DebounceUpdate)
  typeDebounceCnt    ResetDelay;            // Delay for resetting (number of calls to DebounceUpdate)
  uint8              Action;                // See DEBOUNCE_ACTION_xxx
  void*              ActionParam;           // if DEBOUNCE_ACTION_NO this is ignored
                                            // if DEBOUNCE_ACTION_SUB_ERROR this is a pointer to the Sub-Error variable
                                            // if DEBOUNCE_ACTION_LIN_ERROR this is the index of the LIN-Error
}typeDebounceCfg;

/* Declarations #################################################################################################################*/
#ifdef _DEBOUNCE_C_
# define extern
#endif
/* Global variables #############################################################################################################*/
/* Interface-functions ##########################################################################################################*/
extern void DebounceInit( const typeDebounceCfg* pCfg, uint8 NumElements);
extern boolean DebounceUpdate(
  const typeDebounceCfg* pCfg,
  boolean SetImmediate,
  boolean SetDelayed,
  boolean ResetImmediate,
  boolean ResetDelayed);

#ifdef extern
# undef extern
#endif
#endif // _xxx_H_
/*-- End of File --*/

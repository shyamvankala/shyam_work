/**********************************************************************************************************************************
Project     :  L7-Mirror
Filename    :  Ec.h
Brief descr.:  Headerile for Ec (Electrochromic glass) functions (control and diagnostics)
Copyright   :  
Remarks     :  
Version
  1.0.0  18.09.2009  UF  Creation
**********************************************************************************************************************************/
#ifndef _EC_H_
#define _EC_H_

/* Include-Files ################################################################################################################*/
/* Defines ######################################################################################################################*/
/* Type-definitions #############################################################################################################*/
#define EC_STATE_OFF        0
#define EC_STATE_OFF_DELAY  1
#define EC_STATE_CLEARING   2
#define EC_STATE_DIMMED     3

/* Declarations #################################################################################################################*/
#ifdef _EC_C_
# define extern
#endif
/* Global variables #############################################################################################################*/
extern uint8 EcState;
/* Interface-functions ##########################################################################################################*/
extern void EcInit( void);
extern void EcCyclic( void);

uint8_t EC_HS_LS_OVERCURRENT(void);
#define EcGetState() EcState
#define EcIsHighsideActive() ((EcGetState())==EC_STATE_DIMMED)

# if _DEBUG==ON // for debugging only  
extern const typeErrorState* EcGetErrors( void);
extern uint8 EcGetDigStat( void);
# endif
uint8_t EC_PERCENT_TO_DAC_VALUE( uint8_t percent);
#ifdef extern
# undef extern
#endif
#endif // _xxx_H_
/*-- End of File --*/

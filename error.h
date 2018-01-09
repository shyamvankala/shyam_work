/**********************************************************************************************************************************
Project     :  L7-Mirror
Filename    :  Error.h
Brief descr.:  Headerfile for Errorhandling (arrays for suberrors)
Copyright   :  
Remarks     :  
Version
  1.0.0  17.10.2009  UF  Creation
**********************************************************************************************************************************/
#ifndef _ERROR_H_
#define _ERROR_H_

/* Include-Files ################################################################################################################*/
/* Defines ######################################################################################################################*/

// Values for SubErrorState
#define ERROR_STATE_OK      0
#define ERROR_STATE_ERROR   1
#define ERROR_STATE_UNKNOWN 3

/* Type-definitions #############################################################################################################*/
typedef uint8_t typeErrorState;
typedef uint8 typeErrorIdx;
/* Declarations #################################################################################################################*/
#ifdef _ERROR_C_
# define extern
#endif
/* Global variables #############################################################################################################*/
/* Interface-functions ##########################################################################################################*/
extern void ErrorArrayInit( typeErrorState* pErrArray, typeErrorIdx ErrArraySize);
extern uint8 ErrorArrayUpdateResult( typeErrorState* pErrArray, typeErrorIdx ErrArraySize, typeErrorIdx LinErrIdx);

#ifdef extern
# undef extern
#endif
#endif // _xxx_H_
/*-- End of File --*/

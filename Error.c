/* ============================================================================
 * Copyright Information
 * 
 * The content of this file should not be copied, edited, distributed, used
 * without prior authorized permission from SMR Automotive Systems India Ltd.
 * 
 * (C) November 2016
 * ============================================================================
 */

/** ============================================================================
 *  @file      Error.c
 *
 *  @desc      Sourcefile for Errorhandling (arrays for suberrors) 
 *
 *  @document  Integrated_LINBus_Software_Design.docx
 *
 * =============================================================================
 *  Revision History
 *  ===============
 *   Date    Initials   Version   	Change
 * 07-03-17	   APR		 0.0.1    	Initial version
 
 * =============================================================================
*/
#define _ERROR_C_

#include "std_inc.h"
#include "LinAppl.h"

/**
  * @brief  This function Initializes a arror array. All suberrors are set to ERROR_STATE_UNKNOWN.
  * @param  typeErrorState* pErrArray: pointer to an error-array
  *         typeErrorIdx ErrArraySize: number of elements in pErrArray
  * @retval None
*/
void ErrorArrayInit( typeErrorState* pErrArray, typeErrorIdx ErrArraySize){
  typeErrorIdx i;
  for( i = 0; i < ErrArraySize; i++){ /* Reset all errorstates to unknown */
    pErrArray[i] = ERROR_STATE_UNKNOWN;
  }
}
/**
  * @brief  In this function all suberrors are evaluated and a final result is returned.
  *         Priority 1: If at least one error is active, the result is ERROR.
  *         Priority 2: If at least one error is unknown, the result is UNKNOWN.
  *         Only if all suberror are "OK", the result is OK.
  * @param  typeErrorState* pErrArray: pointer to an error-array
  *         typeErrorIdx ErrArraySize: number of elements in pErrArray
  *         typeErrorIdx LinErrIdx   : Error index to be used as parametert to l_u8_wr_ERR_ST_EXMI_XH_LIN.
  * @retval uint8: result of checking all suberror: ERROR_STATE_UNKNOWN/LIN_ERR_OK/ERROR_STATE_ERROR
*/
uint8 ErrorArrayUpdateResult( typeErrorState* pErrArray, typeErrorIdx ErrArraySize, typeErrorIdx LinErrIdx){
  uint8 Result = LIN_ERR_OK;
  typeErrorIdx i;
  for( i = 0; i < ErrArraySize; i++){
    if( pErrArray[i] == ERROR_STATE_ERROR){
      Result = LIN_ERR_ERR;
      break;
    }
    if( pErrArray[i] == ERROR_STATE_UNKNOWN){
      Result = LIN_ERR_SNV;
    }
  }
  l_u8_wr_ERR_ST_EXMI_XH_LIN( LinErrIdx, Result);
  return Result;
}


/*-- End of File --*/

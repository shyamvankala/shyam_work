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
 *  @file      Debounce.c
 *
 *  @desc      Sourcefile for debounce functionality 
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
#define _DEBOUNCE_C_

#include "std_inc.h"
#include "debounce.h"
#include "LinAppl.h"

#define DEBOUNCE_STATE_RESET      0
#define DEBOUNCE_STATE_SETDELAY   1
#define DEBOUNCE_STATE_SET        2
#define DEBOUNCE_STATE_RESETDELAY 3

/**
  * @brief  This function Initialize an array of debounce data structures to reset value (=RESET, no stable result detected)
  * @param  const typeDebounceCfg* pCfg: pointer to debounce configuration array
  *         uint8 NumElements          : Number of elements in the array
  * @retval None
*/
void DebounceInit( const typeDebounceCfg* pCfg, uint8 NumElements){
  uint8_least i;
  for( i = 0; i < NumElements; i++){
    pCfg[i].pState->State = DEBOUNCE_STATE_RESET;
    pCfg[i].pState->StableResultDet = FALSE;
  }  
}

/**
  * @brief  This function Updates the state of a debounce statemachine specified by pCfg with the given conditions. 
  * @param  const typeDebounceCfg* pCfg: Pointer to configuration
  *         boolean SetImmediate       : Condition for immediate detection of failure is active
  *         boolean SetDelayed         : Condition for delayed detection of failure is active
  *         boolean ResetImmediate     : Condition for immediate reset of failure is active
  *         boolean ResetDelayed       : Condition for delayed reset of failure is active
  * @retval None
*/
boolean DebounceUpdate(
  const typeDebounceCfg* pCfg,
  boolean SetImmediate,
  boolean SetDelayed,
  boolean ResetImmediate,
  boolean ResetDelayed){
  boolean fNewStableState = FALSE;

  // Check for state transitions
  if( ResetImmediate){
    if( (pCfg->pState->State == DEBOUNCE_STATE_SET) || (pCfg->pState->State == DEBOUNCE_STATE_RESETDELAY)){
      fNewStableState = TRUE;
    }
    pCfg->pState->State = DEBOUNCE_STATE_RESET;
  }
  else if( SetImmediate){
    if( (pCfg->pState->State == DEBOUNCE_STATE_RESET) || (pCfg->pState->State == DEBOUNCE_STATE_SETDELAY)){
      fNewStableState = TRUE;
    }
    pCfg->pState->State = DEBOUNCE_STATE_SET;
  }
  else if( (pCfg->pState->State == DEBOUNCE_STATE_RESET) && SetDelayed){
    if( pCfg->SetDelay == 0){
      pCfg->pState->State = DEBOUNCE_STATE_SET;
      fNewStableState = TRUE;
    }
    else{
      pCfg->pState->State = DEBOUNCE_STATE_SETDELAY;
      *(pCfg->pCounter) = 0;
    }
  }


  else if( (pCfg->pState->State == DEBOUNCE_STATE_SET) && ResetDelayed){
    if( pCfg->ResetDelay == 0){
      pCfg->pState->State = DEBOUNCE_STATE_RESET;
      fNewStableState = TRUE;
    }
    else{
      pCfg->pState->State = DEBOUNCE_STATE_RESETDELAY;
      *(pCfg->pCounter) = 0;
    }
  }
  else if( pCfg->pState->State == DEBOUNCE_STATE_SETDELAY){
    if( !SetDelayed){
      pCfg->pState->State = DEBOUNCE_STATE_RESET;
    }
    else if( ++(*(pCfg->pCounter)) > pCfg->SetDelay){
      pCfg->pState->State = DEBOUNCE_STATE_SET;
      fNewStableState = TRUE;
    }
  }
  else if( pCfg->pState->State == DEBOUNCE_STATE_RESETDELAY){
    if( !ResetDelayed){
      pCfg->pState->State = DEBOUNCE_STATE_SET;
    }
    else if( ++(*(pCfg->pCounter)) > pCfg->ResetDelay){
      pCfg->pState->State = DEBOUNCE_STATE_RESET;
      fNewStableState = TRUE;
    }
  }
  // Check for new stable state or first time with defined state
  if( fNewStableState || (pCfg->pState->StableResultDet == FALSE)){
    if( pCfg->pState->State == DEBOUNCE_STATE_RESET){
      if( pCfg->Action == DEBOUNCE_ACTION_SUB_ERROR){
        *(typeErrorState*)(pCfg->ActionParam) = ERROR_STATE_OK;
      }
      else if( pCfg->Action == DEBOUNCE_ACTION_LIN_ERROR){      
        l_u8_wr_ERR_ST_EXMI_XH_LIN( (uint8)(uint16)(pCfg->ActionParam), LIN_ERR_OK);
      }
      pCfg->pState->StableResultDet = TRUE;
    }
    else if( pCfg->pState->State == DEBOUNCE_STATE_SET){
      if( pCfg->Action == DEBOUNCE_ACTION_SUB_ERROR){
        *(typeErrorState*)(pCfg->ActionParam) = ERROR_STATE_ERROR;
      }
      else if( pCfg->Action == DEBOUNCE_ACTION_LIN_ERROR){      
        l_u8_wr_ERR_ST_EXMI_XH_LIN( (uint8)(uint16)(pCfg->ActionParam), LIN_ERR_ERR);
      }
      pCfg->pState->StableResultDet = TRUE;
    }
  }
  return (pCfg->pState->State == DEBOUNCE_STATE_SET) || (pCfg->pState->State == DEBOUNCE_STATE_RESETDELAY);
}

/*-- End of File --*/


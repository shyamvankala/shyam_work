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
 *  @file      Mmot.c
 *
 *  @desc      Sourcefile for Glass Drive and memory functions (control and diagnostics) 
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
#define _MMOT_C_

#include "Mmot.h"
#include "ZMDCM0.h"
#include "SPI.h"
#include "ADC.h"
#include "SBC.h"
#include "SPIMgr.h"
#include "LINCom.h"
#include "SBC_cfg.h"
#include "LINCom_cfg.h"
#include "HS_cfg.h"
#include "Hardware.h"
#include "HS.h"
#include "HB.h"
#include "PWM.h"
#include "Applinit.h"
#include "Vmon.h"
#include "Tmon.h"
#include "pfld.h"
#include "error.h"
#include "timer.h"
#include "params.h"
#include "LinAppl.h"
#include "applinit.h"

#ifdef _LEFT_MIRROR // left mirror
# define SELECTED_FOR_MEMORY() (l_u8_rd_CTR_EXMI_MEMO_LIN()==LIN_MPOT_SELECT_ASP_LEFT)
#endif
#ifdef _RIGHT_MIRROR // right mirror
# define SELECTED_FOR_MEMORY() (l_u8_rd_CTR_EXMI_MEMO_LIN()==LIN_MPOT_SELECT_ASP_RIGHT)
#endif
// Get potentiometer value for actual axis (x or y)
#define POTI_VALUE_FOR_ACTUAL_DIRECTION()  (MmotIsDirectionX()? l_u8_rd_PO_AVL_EXMI_XH_AX_RH_LH_LIN():l_u8_rd_PO_AVL_EXMI_XH_AX_UP_DWN_LIN())
// assignment of suberror-index
#define MMOT_ERRIDX_OVERCURRENT_LEFT   0
#define MMOT_ERRIDX_OVERCURRENT_RIGHT  1
#define MMOT_ERRIDX_OPENLOAD_X         2
#define MMOT_ERRIDX_OVERCURRENT_UP     3
#define MMOT_ERRIDX_OVERCURRENT_DOWN   4
#define MMOT_ERRIDX_OPENLOAD_Y         5
#if HBDIAG_EXTENDED==OFF
#define MMOT_NUM_SUB_ERRORS            6
#else
#define MMOT_ERRIDX_HBEXTENDED         6 // Extended halfbridge diagnostics error
#define MMOT_NUM_SUB_ERRORS            7
#endif
#define POTI_MID_POS (uint8)127
#define POTI_DELTA_TO_POS( potival, pos) (uint8)((potival)>=(pos)?(potival)-(pos):(pos)-(potival))
#define MMOT_CURR_MAX_AMPS_NEW		   90
// Values for typeMmState.sub.memory
#define MMOT_MANUAL                    0
#define MMOT_MEMORY                    1

// Table to retrieve the HS-Output of the L99MM70 for a specific direction.
// This is required for current measurement
typedef struct{
  uint16 OverCurrLsbs; // Overcurrent in scaling to ADC
  //uint16 HbMask;     // Mask for Halfbridge-Errors (used for open-load)
  uint8 OutNr;
  uint8 ErrIdxOc;      // ErrorIndex for overcurrent
  uint8 ErrIdxOpnLoad; // ErrorIndex for open load
}typeMmotDirTbl;

extern const uint8 LIN_CTRL_SELECT_ASP_XXXX;
extern uint8_t bVTFailure;

uint8 MemoryNewGotoX, MemoryNewGotoY;
uint8 Memory_Request;
boolean MemoryNewRequest;
 
uint16 mmot_curr_raw;

static typeErrorState aMmotSubErrorState[MMOT_NUM_SUB_ERRORS];
static const typeMmotDirTbl aMmotDirTbl[4] = { {MMOT_CURR_MAX_AMPS_NEW, MMOT_LEFT , MMOT_ERRIDX_OVERCURRENT_LEFT , MMOT_ERRIDX_OPENLOAD_X},
											   {MMOT_CURR_MAX_AMPS_NEW, MMOT_RIGHT, MMOT_ERRIDX_OVERCURRENT_RIGHT, MMOT_ERRIDX_OPENLOAD_X},
											   {MMOT_CURR_MAX_AMPS_NEW, MMOT_UP   , MMOT_ERRIDX_OVERCURRENT_UP   , MMOT_ERRIDX_OPENLOAD_Y},
											   {MMOT_CURR_MAX_AMPS_NEW, MMOT_DOWN , MMOT_ERRIDX_OVERCURRENT_DOWN , MMOT_ERRIDX_OPENLOAD_Y}
											 };	  	// Mirrormovement - direction table	

void MemoryStart( uint8 GotoX, uint8 GotoY);

static void MotorStart( uint8 direction, boolean mode);
static void MotorBrake( void);
static uint16 PotiDeltaToMidPosSquared( uint8 potival);
static void MmotUpdateOvercurrentError( uint8 state);
static uint16 MmotCtr; // For time-measurement run (timeout) and brake
static uint16 MmotStallCtr; // For detection of blocking motor (no ADC change on potentiometer)
static uint8 MemoryGotoX, MemoryGotoY; // Requested values for memory
static uint8 poti_last; // for motor-stall detection
static typeTimerValue MmotOvercurrentCtr;
static boolean MmotOpenLoad;

extern uint8_t u8HBOutputState;
/**
  * @brief  This function initializes all variables to reset state.
  * @param  None
  * @retval None
*/
void MmotInit( void){
  //setting mmot state machine to IDLE state
  MmotState.sub.state = MMOT_STATE_IDLE;
  //Initializing error array
  ErrorArrayInit( aMmotSubErrorState, MMOT_NUM_SUB_ERRORS);
  Memory_Request = 0;
}

/**
  * @brief  This function Must be called cyclically every MMOT_CYCLE_TIME_MS.
  *         Updates application (checks inputs and updates statemachine and outputs) and diagnostic functions for mirror
  * @param  None
  * @retval None
*/
void MmotCyclic( void){
  uint8 ManAdjDir = MMOT_DIR_OFF; // in case of error or off
  // First get manual adjustment command from LIN
  if( (l_u8_rd_BRC_CTR_EXMI_LIN()==LIN_CTRL_SELECT_ASP_XXXX)||
    (l_u8_rd_BRC_CTR_EXMI_LIN()==LIN_CTRL_SELECT_ASP_BOTH)){
    switch( l_u8_rd_BRC_CTR_EXMI_AD_LIN()){
    case LIN_CTRL_MMOT_ADJUST_LEFT:
      ManAdjDir = MMOT_DIR_LEFT;
      break;
    case LIN_CTRL_MMOT_ADJUST_RIGHT:
      ManAdjDir =  MMOT_DIR_RIGHT;
      break;
    case LIN_CTRL_MMOT_ADJUST_UP:
      ManAdjDir = MMOT_DIR_UP;
      break;
    case LIN_CTRL_MMOT_ADJUST_DOWN:
      ManAdjDir = MMOT_DIR_DOWN;
      break;
   }
  }
  // Update statemachine
  if( VTFailure() || PfldIsActive()) // No statechange
   { // memory request was received during Powerfold
	    if( LinApplUpdateBrcMemPos() == E_OK){ 
		      if( SELECTED_FOR_MEMORY()){
		        MemoryNewGotoX = l_u8_rd_PO_EXMI_AX_LH_RH_LIN();
		        MemoryNewGotoY = l_u8_rd_PO_EXMI_AX_UP_DWN_LIN();
		        Memory_Request = 1; // Memory request during powerfold
	     		}    
    	}
    	if( MmotState.sub.state == MMOT_STATE_RUNNING){
      		MmotState.sub.state = MMOT_STATE_SUSPEND;
    		}
    	else if((MmotState.sub.state == MMOT_STATE_SUSPEND) && (MmotState.sub.memory == MMOT_MEMORY)){
      		// if in suspended state and memory mode a manual adjustment is detected, the memory must be aborted
      		// otherwise it is continued after changing from supend to running
      		if( ManAdjDir != MMOT_DIR_OFF){
        		MmotState.sub.state = MMOT_STATE_IDLE;
     		 }
    	}
  	}
    else if( MmotState.sub.state == MMOT_STATE_BRAKING){ // Check, if braking is finished
    	if( MMOT_OVERCURRENT()){ // Overcurrent flag ZAMC4200
      		MmotUpdateOvercurrentError( ERROR_STATE_ERROR);

    		}
    	else if( ++MmotCtr >= MMOT_BRAKE_TIME_MS/MMOT_CYCLE_TIME_MS){
     		 MmotUpdateOvercurrentError( ERROR_STATE_OK);
   			}
    	else{ // continue with braking
      			return;
   			 }
    	if( MmotState.sub.error == TRUE){ // First condition -> that's why the second memory axis is not moved if the first axis has an error. Behaviour can be changed by checking for memory before error.
    	// UK Change for second memory cycle (L), in case the target is not reached during the first cycle (L)
     	 if ((MmotState.sub.mem_x_ok == FALSE)||(MmotState.sub.mem_y_ok == FALSE)) {
        	if (((MmotState.sub.dir == MMOT_DIR_UP) || (MmotState.sub.dir == MMOT_DIR_DOWN)) && (MmotState.sub.mem_rep_x)) {
          	if( MemoryGotoX > l_u8_rd_PO_AVL_EXMI_XH_AX_RH_LH_LIN()){ // actual position is too left => move to right
            MotorStart( MMOT_DIR_RIGHT, MMOT_MEMORY);
           }
           else{  // too right
            MotorStart( MMOT_DIR_LEFT, MMOT_MEMORY);
           }
           MmotState.sub.mem_rep_x = FALSE;
        }
        else if (((MmotState.sub.dir == MMOT_DIR_LEFT) || (MmotState.sub.dir == MMOT_DIR_RIGHT)) && (MmotState.sub.mem_rep_y)) {
          if( MemoryGotoY > l_u8_rd_PO_AVL_EXMI_XH_AX_UP_DWN_LIN()){ // actual position is too down => move up
            MotorStart( MMOT_DIR_UP, MMOT_MEMORY);
          }
          else{  // too up
            MotorStart( MMOT_DIR_DOWN, MMOT_MEMORY);
          }
          MmotState.sub.mem_rep_y = FALSE;
        } 
        else {
          MmotState.sub.mem_x_ok = TRUE;
          MmotState.sub.mem_y_ok = TRUE;
        }
      } 
      else {
        MmotState.sub.state = MMOT_STATE_ERROR;        
      }
    }
    else if( MmotState.sub.memory == MMOT_MANUAL){ // no memory -> finsihed
      MmotState.sub.state = MMOT_STATE_IDLE;
    }
    else{ // Memory active
      // Memory Axis adjustment completed, check if ready or start next axis
      if( MmotIsDirectionX()){ // X-Axis OK
        MmotState.sub.mem_x_ok = TRUE;    
      }
      else{ // Y-Axis OK
        MmotState.sub.mem_y_ok = TRUE;
      }
      if( MmotState.sub.mem_x_ok == FALSE){
        if( MemoryGotoX > l_u8_rd_PO_AVL_EXMI_XH_AX_RH_LH_LIN()){ // too left
          MotorStart( MMOT_DIR_RIGHT, MMOT_MEMORY);
        }
        else{  // too right
          MotorStart( MMOT_DIR_LEFT, MMOT_MEMORY);
        }
      }
      else if( MmotState.sub.mem_y_ok == FALSE){
        if( MemoryGotoY > l_u8_rd_PO_AVL_EXMI_XH_AX_UP_DWN_LIN()){ // too down
          MotorStart( MMOT_DIR_UP, MMOT_MEMORY);
        }
        else{  // too up
          MotorStart( MMOT_DIR_DOWN, MMOT_MEMORY);
        }
      }
      else{ // both OK -> memory completed    
        MmotState.sub.state = MMOT_STATE_IDLE;
      }
    }
  }
  else{
    // Update memory request always first
    // Either this is to be taken and the statemachine starts to go to the target position or it is ignored.(if also a manual request is present)
     MemoryNewRequest = FALSE;

    // If from suspended goto "running"
    if( MmotState.sub.state == MMOT_STATE_SUSPEND){
      MmotState.sub.state = MMOT_STATE_RUNNING;
    }

    // Ckeck for memory request
    if( LinApplUpdateBrcMemPos() == E_OK){ // memory request was received
      if( SELECTED_FOR_MEMORY()){
        MemoryNewGotoX = l_u8_rd_PO_EXMI_AX_LH_RH_LIN();
        MemoryNewGotoY = l_u8_rd_PO_EXMI_AX_UP_DWN_LIN();
	//	MemoryStart( MemoryNewGotoX, MemoryNewGotoY);
        MemoryNewRequest = TRUE;
      }    
    }
   // Check for memory request during active powerfold
    else if (Memory_Request == 1){
      MemoryNewRequest = TRUE;
    }
    if( MmotState.sub.state == MMOT_STATE_IDLE){
      if( ManAdjDir != MMOT_DIR_OFF){
        MotorStart( ManAdjDir, MMOT_MANUAL);
      }
      else if( MemoryNewRequest){ // If no manual activation request is active, check for memory request
		 Memory_Request = 0;
        MemoryStart( MemoryNewGotoX, MemoryNewGotoY);
      }
    }
    else if( MmotState.sub.state == MMOT_STATE_ERROR){
      // Wait for off or change of direction
       if( ManAdjDir == MMOT_DIR_OFF){
        MmotState.sub.state = MMOT_STATE_IDLE;
      }
      else if( ManAdjDir != MmotGetDirection()){ // Change of direction
        MotorStart( ManAdjDir, MMOT_MANUAL);
      }
    }
    else{ // Mm already active
      boolean new_timeout_error = FALSE;
     
      const typeMmotDirTbl* pMmotDirTbl = &(aMmotDirTbl[MmotGetDirection()]);
	   #if 1
      // Overcurrent detection
      if( ZMDIApplReadCurrentMonitorRaw( pMmotDirTbl->OutNr, &mmot_curr_raw) == E_OK){
        if( mmot_curr_raw > pMmotDirTbl->OverCurrLsbs){
          MmotOvercurrentCtr++; // Count is incremented if valid result is available and I > max
        }
        else{
          MmotOvercurrentCtr = 0; // Reset counter, if current is below limit
        }
      }
	  #endif
      if( MMOT_OVERCURRENT() || 
         (MmotOvercurrentCtr > (MMOT_CURR_MAX_TIME_MS/MMOT_CYCLE_TIME_MS))){  // Overcurrent on highside
          MmotState.sub.error = TRUE;
          MmotState.sub.state = MMOT_STATE_ERROR;
          MmotUpdateOvercurrentError( ERROR_STATE_ERROR);
			SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(HBDSTAT))[0] =  u8HBOutputState|0x0E;	
			SPIMgr_eRegWrite(HBDSTAT); 
		 // MotorBrake();
      }
      else{
//#if _EMC_TEST==OFF // Timeouts are not checked with EMC-Tests
        // Check for Timeout since start of movement
        MmotCtr++;
        if( MmotCtr > (MMOT_TOTAL_TIME_MAX_MS/MMOT_CYCLE_TIME_MS)){
          new_timeout_error = TRUE;
        }
        else if( POTI_DELTA_TO_POS( POTI_VALUE_FOR_ACTUAL_DIRECTION(), poti_last) >= MPOT_MIN_CHANGE_RUN_LSBS){
          poti_last = POTI_VALUE_FOR_ACTUAL_DIRECTION();
          MmotStallCtr = 0;
          MmotState.sub.moved = TRUE;
        }
        else{
          MmotStallCtr++;
          if( MmotState.sub.moved == FALSE){ // No move detected up to now -> take long timeout
            if( MmotStallCtr > (MMOT_STALL_DET_INIT_MS/MMOT_CYCLE_TIME_MS)){
        //      new_timeout_error = TRUE;
		//	  MotorBrake();
            }
          }
          else{ // Move was detected before -> take short timeout
            if( MmotStallCtr > MMOT_STALL_DET_RUN_MS/MMOT_CYCLE_TIME_MS){
              new_timeout_error = TRUE;
            }
          }
        }
//#else
//#warning EMC test mode is active!
//#endif
        
        // Open Load Detection
        if(!MMOT_OPEN_LOAD()){
          MmotOpenLoad = FALSE; // Clear if not all open-load flags are set (accumulation over complete run-phase)
          aMmotSubErrorState[pMmotDirTbl->ErrIdxOpnLoad] = ERROR_STATE_OK; // reset error immediately
        }
        
        // Check if timeout error: if this is the case, check also for open load and change state to error
        if( new_timeout_error == TRUE){
          aMmotSubErrorState[pMmotDirTbl->ErrIdxOpnLoad] = MmotOpenLoad? ERROR_STATE_ERROR : ERROR_STATE_OK;
          MmotState.sub.error = TRUE;		  
          MotorBrake();
        }
        else{ // no timeout error
          if( MmotState.sub.memory == FALSE){ // manual adjustment
            if( ManAdjDir == MMOT_DIR_OFF){
			
              MotorBrake();
            }
            else if( ManAdjDir != MmotGetDirection()){ // Change of direction
              MotorStart( ManAdjDir, MMOT_MANUAL);
            }
          }      
          else{ // memory adjustment
            if( ManAdjDir != MMOT_DIR_OFF){ // memory is aborted in case of a manual adjustment
              MotorStart( ManAdjDir, MMOT_MANUAL);
            }
            else{
              if( MemoryNewRequest && ((MemoryGotoX != MemoryNewGotoX) || (MemoryGotoY != MemoryNewGotoY))){ // Restart only if target coordinates are different
                  MemoryStart( MemoryNewGotoX, MemoryNewGotoY);
              }
              // Check if the target position is near and the brake has to be activated
              else if( (MmotGetDirection() == MMOT_DIR_RIGHT) && (l_u8_rd_PO_AVL_EXMI_XH_AX_RH_LH_LIN() >= (MemoryGotoX-MPOT_BRAKE_BEFORE_LSBS)) ||
                  (MmotGetDirection() == MMOT_DIR_LEFT ) && (l_u8_rd_PO_AVL_EXMI_XH_AX_RH_LH_LIN() <= (MemoryGotoX+MPOT_BRAKE_BEFORE_LSBS)) ||
                  (MmotGetDirection() == MMOT_DIR_UP   ) && (l_u8_rd_PO_AVL_EXMI_XH_AX_UP_DWN_LIN() >= (MemoryGotoY-MPOT_BRAKE_BEFORE_LSBS)) ||
                  (MmotGetDirection() == MMOT_DIR_DOWN ) && (l_u8_rd_PO_AVL_EXMI_XH_AX_UP_DWN_LIN() <= (MemoryGotoY+MPOT_BRAKE_BEFORE_LSBS))){
                MotorBrake();              
              }
            }
          }
        }
      }       
    }
  }
  //Updating the mmot error array 
  ErrorArrayUpdateResult( aMmotSubErrorState, MMOT_NUM_SUB_ERRORS, LIN_ERR_IDX_MMOT);
}





/**
  * @brief  This function set halfbridge extended error state
  * @param  typeErrorState State: New error state
  * @retval None
*/
#if HBDIAG_EXTENDED==ON
void MmotSetHbExtDiagErrorState( typeErrorState State){
  aMmotSubErrorState[MMOT_ERRIDX_HBEXTENDED] = State;
}
#endif
/**
  * @brief  This function returns a pointer to the internal diagnostic information - for debugging only.
  * @param  None
  * @retval const typeErrorState*: pointer to internal errorstates.
*/
#if _DEBUG==ON // for debugging only  
const typeErrorState* MmotGetErrors( void){
  return aMmotSubErrorState;
}
#endif
/**
  * @brief  This function Performs all required actions to start the mirrorglass motor.
  * @param  uint8 direction: direction for movement: MMOT_DIR_RIGHT/MMOT_DIR_LEFT/MMOT_DIR_UP/MMOT_DIR_DOWN
  *         boolean mode : memory (MMOT_MEMORY) or manual (MMOT_MANUAL)
  * @retval None
*/
static void MotorStart( uint8 direction, boolean mode){
  MmotState.sub.state = MMOT_STATE_RUNNING;
  MmotState.sub.dir = direction;
  MmotState.sub.memory = mode;
  MmotState.sub.moved = FALSE;
  MmotState.sub.error = FALSE;
  
  // Store potentiometer value for detection of movement
  poti_last = POTI_VALUE_FOR_ACTUAL_DIRECTION();
  MmotStallCtr = 0;
  
  MmotCtr = 0; // for total timeout
  
  MmotOpenLoad = TRUE; // Set at start. If one of the open-load flags is cleared at least once -> no open load error
  MmotOvercurrentCtr = 0;
}
/**
  * @brief  This function Performs all required actions to brake the mirrorglass motor.
  * @param  None
  * @retval None
*/
static void MotorBrake( void){
  MmotState.sub.state = MMOT_STATE_BRAKING;
  //ZMDIMmotHBBrake();
  MmotCtr = 0; // for brake time
  MmotUpdateOvercurrentError( ERROR_STATE_OK);
}
/**
  * @brief  This function Performs all required actions to start the mirrorglass motor if memory was requested.
  *         The first direction for movement is calculated taking care on "dead zones" and the motor is started.
  * @param  uint8 GotoX: Target coordinate X
  *         uint8 GotoY: Target coordinate Y
  * @retval None
*/
static void MemoryStart( uint8 GotoX, uint8 GotoY){
  MemoryGotoX = GotoX;
  MemoryGotoY = GotoY;
  
  MmotState.sub.mem_x_ok = (MemoryGotoX == LIN_POTI_SNV) ||
    (MemoryGotoX <= (l_u8_rd_PO_AVL_EXMI_XH_AX_RH_LH_LIN()+MPOT_MIN_DEVIATION_LSBS) &&
     MemoryGotoX >= (l_u8_rd_PO_AVL_EXMI_XH_AX_RH_LH_LIN()-MPOT_MIN_DEVIATION_LSBS));
  MmotState.sub.mem_y_ok = (MemoryGotoY == LIN_POTI_SNV) ||
    (MemoryGotoY <= (l_u8_rd_PO_AVL_EXMI_XH_AX_UP_DWN_LIN()+MPOT_MIN_DEVIATION_LSBS) &&
     MemoryGotoY >= (l_u8_rd_PO_AVL_EXMI_XH_AX_UP_DWN_LIN()-MPOT_MIN_DEVIATION_LSBS)); 
  
  MmotState.sub.mem_rep_x = !MmotState.sub.mem_x_ok;
  MmotState.sub.mem_rep_y = !MmotState.sub.mem_y_ok;
  
  // The movement must go as near to the mirror middle as possible due to unreachable areas in the corners
  // start with X, if 
  // X Position is not at target position and  DELTAmid_gotox + DELTAmid_starty is smaller than DELTAmid_gotoy + DELTAmid_startx
  // This is a simplified calculation like sentence of pythagoras (a²=b²+c²) for the turning point.
  if( MmotState.sub.mem_x_ok == FALSE && // Start only if X has to be moved
    ((MmotState.sub.mem_y_ok == TRUE) || // if Y is ok we always have to start with X
    ((PotiDeltaToMidPosSquared( MemoryGotoX) + PotiDeltaToMidPosSquared( l_u8_rd_PO_AVL_EXMI_XH_AX_UP_DWN_LIN())) < 
    (PotiDeltaToMidPosSquared( MemoryGotoY) + PotiDeltaToMidPosSquared( l_u8_rd_PO_AVL_EXMI_XH_AX_RH_LH_LIN()))))){ // If movement is started with X the turnpoint is nearer to the center
    // Start with x
    if( MemoryGotoX > l_u8_rd_PO_AVL_EXMI_XH_AX_RH_LH_LIN()){ // actual position is too left => move to right
      MotorStart( MMOT_DIR_RIGHT, MMOT_MEMORY);
    }
    else{  // too right
      MotorStart( MMOT_DIR_LEFT, MMOT_MEMORY);
    }
  }										 
  else if( MmotState.sub.mem_y_ok == FALSE){ // Start with y
    if( MemoryGotoY > l_u8_rd_PO_AVL_EXMI_XH_AX_UP_DWN_LIN()){ // actual position is too down => move up
      MotorStart( MMOT_DIR_UP, MMOT_MEMORY);
    }
    else{  // too up
      MotorStart( MMOT_DIR_DOWN, MMOT_MEMORY);
    }
  }
  else{ // no adjustment required    
  }
}
/**
  * @brief  This function calculate the unsigned difference between this coordinate (x or y) and the middle of the mirror. This difference
  *         is squared and returned.This function is used for decision of first direction (x or y) with memory.
  *         Therefore an approach with sentence of pythagoras (c²=a²+b²) is used.The returned value is (127-potival)².
  * @param  uint8 potival: coordinate (x or y) for which the difference shall be calculated
  * @retval uint16: square of difference between potival and middle
*/
static uint16 PotiDeltaToMidPosSquared( uint8 potival){
  uint8 dist;
  dist = potival>=POTI_MID_POS? (uint8)(potival - POTI_MID_POS) : (uint8)(POTI_MID_POS - potival);
  return dist*dist;
}

/**
  * @brief  This function updates the overcurrent error state for the actual direction
  * @param  uint8 state: new overcurrent state
  * @retval None
*/
static void MmotUpdateOvercurrentError( uint8 state){
	  aMmotSubErrorState[aMmotDirTbl[MmotGetDirection()].ErrIdxOc] = state;
}
/**
  * @brief  This function check for the overcurrent flag set/clear value
  * @param  None
  * @retval 1 - OC Flag is set
  *			0 - OC Flag is not set
*/

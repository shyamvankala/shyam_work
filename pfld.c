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
 *  @file      pfld.c
 *
 *  @desc      Sourcefile for Powerfold function (control and diagnostics) 
 *
 *  @document  Integrated_LINBus_Software_Design.docx
 *
 * =============================================================================
 *  Revision History
 *  ===============
 *   Date    Initials   Version   	Change
 * 07-03-17	   BG		 0.0.1    	Initial version
 
 * =============================================================================
*/
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
#include "pfld.h"
#include "TMR32_Cfg.h"
#include "TMR32.h"
#include "HB.h"
#include "Applinit.h"
#include "params.h"
#include "error.h"
#include "GPIO.h"
#include "LinAppl.h"

// Values for BRC_CTR_EXMI_FMIR_LIN
#define LIN_CTRL_PFLD_NO      0
#define LIN_CTRL_PFLD_DRIVE   2
#define LIN_CTRL_PFLD_PARK    1
#define LIN_CTRL_PFLD_SNV     3
#define PFLD_ERRIDX_OPENLOAD  2 // open load error (for both directions)
#define PFLD_ERRIDX_OVERCURRENT_DRIVE 0 // overcurrent during moving to "drive" position
#define PFLD_ERRIDX_OVERCURRENT_PARK  1 // overcurrent during moving to "park" position
#define PFLD_NUM_SUB_ERRORS   3			//BR changed from 4 to 3 to fix diagnostic error

l_bool now_at_block ;
uint16_t pfld_curr_peak,pfld_curr_block;
//uint16_t pfld_curr_raw;
uint16_t pfld_direction ,Ubat ;
uint16_t pfld_curr_peak_save;
uint16_t MaxontimeCtr;
uint8_t pfld_PRESENT_direction ;
uint8_t pfld_active_flag;
uint16_t  count ,newVal;  
uint8_t flag;
uint16_t  oldVal;
uint8_t setFlag;
uint8_t u8HBOutputState;

static typeTimerValue CtrlCtr;
static typeTimerValue RunCtr;
static typeTimerValue InRushCurrentCtr;
static typeTimerValue block_counter;
static typeTimerValue PfldOvercurrentCtr;
static typeTimerValue TimeoutTicks;
static boolean PfldOpenLoad;
static typeTimerValue GetBlockTime( void);
static typeTimerValue GetMaxOnTime( void);
static typeErrorState aPfldSubErrorState[PFLD_NUM_SUB_ERRORS];
static void PfldMotorStart( uint8 direction);
static uint8_t * HBstat,*IRQstat;
static uint16_t pfld_current;

/**
  * @brief  This function Initializes all powerfold variables to reset state   
  * @param  None
  * @retval None
*/
void PfldInit( void){
  PfldState.sub.state = PFLD_STATE_IDLE;
  PfldState.sub.dir_change = OFF;
  PfldState.sub.repeat = OFF;

  // Enable open load diagnostic for powerfold : added by BR 
 // SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(HBDDIAG))[0] =  0x06;
  //SPIMgr_eRegWrite(HBDDIAG);

   ErrorArrayInit( aPfldSubErrorState, PFLD_NUM_SUB_ERRORS);
}
/**
  * @brief  This function Must be called cyclically every PFLD_CYCLE_TIME_MS.
  *         Updates application (checks inputs and updates statemachine and outputs) and diagnostic functions for powerfold. 
  * @param  None
  * @retval None
*/
void PfldCyclic( void){
  uint8 lin_cmd = PFLD_DIR_OFF; // in case of error or off
  uint16 Ubatt;
  uint16 pfld_curr_raw ;
  // First read actual LIN-command
  if( l_u8_rd_BRC_CTR_EXMI_LIN() != LIN_CTRL_SELECT_ASP_SNV){ // Ignore, if BRC_CTR_EXMI_LIN is SNV
    switch( l_u8_rd_BRC_CTR_EXMI_FMIR_LIN()){
		//switch(flag){
    	case LIN_CTRL_PFLD_DRIVE:
      		lin_cmd = PFLD_DIR_DRIVE;
      	break;
    	case LIN_CTRL_PFLD_PARK:
      		lin_cmd = PFLD_DIR_PARK;
      	break;
    }
  }

  // Update states
  if( VTFailure() || (l_u8_rd_BRC_CTR_EXMI_LIN() == LIN_CTRL_SELECT_ASP_SNV)){
  //if( (l_u8_rd_BRC_CTR_EXMI_LIN() == LIN_CTRL_SELECT_ASP_SNV)){
      if(l_u8_rd_BRC_CTR_EXMI_LIN() == LIN_CTRL_SELECT_ASP_SNV){
		if( (PfldState.sub.state == PFLD_STATE_RUNNING) || (PfldState.sub.state == PFLD_STATE_BLOCKING) || (PfldState.sub.state == PFLD_STATE_BRAKING)){
      		PfldState.sub.state = PFLD_STATE_SUSPEND;
    	}
     if( PfldState.sub.state == PFLD_STATE_SUSPEND){
      if( lin_cmd == PFLD_DIR_OFF){ // Avoid continuation of a movement that was started before VT-Failure
        PfldState.sub.state = PFLD_STATE_IDLE;
       }
     }
    else if( PfldState.sub.state == PFLD_STATE_WAITING){
      // stay in "waiting" until the request is off or changed (otherwise the motor is activated after removing VTFailure!
      if( (lin_cmd == PFLD_DIR_OFF) || // now off
          (lin_cmd != PfldGetDirection())){ // direction changed
        PfldState.sub.state = PFLD_STATE_IDLE;
       }
     } 
    }
  }
  else{
    if( PfldState.sub.state == PFLD_STATE_IDLE){
      if(  lin_cmd != PFLD_DIR_OFF){
        PfldMotorStart( lin_cmd);        
      }
      else if( (PfldState.sub.repeat == ON) && (VmonGetUbatRaw() > UBAT_MEAS_VOLTS_TO_LSBS(PFLD_REACTIVATE_TRIGGER))) { 
        PfldState.sub.repeat = OFF;
        PfldMotorStart( PFLD_DIR_DRIVE);
      }
    }
    else if( PfldState.sub.state == PFLD_STATE_WAITING){
      if( lin_cmd == PFLD_DIR_OFF){ // now off
        PfldState.sub.state = PFLD_STATE_IDLE;
      }
      else if( lin_cmd != PfldGetDirection()){ // direction changed
        PfldMotorStart( lin_cmd);        
      } 
      else if( (PfldState.sub.repeat == ON) && (VmonGetUbatRaw() > UBAT_MEAS_VOLTS_TO_LSBS(PFLD_REACTIVATE_TRIGGER))) { 
        PfldState.sub.repeat = OFF;
        PfldMotorStart( PFLD_DIR_DRIVE);
      }
    }
    else{ // running or blocking or braking
      // If from suspended goto "running"
      if( PfldState.sub.state == PFLD_STATE_SUSPEND){
        PfldState.sub.state = PFLD_STATE_RUNNING;
      }
      if( (lin_cmd != PFLD_DIR_OFF) && (lin_cmd != PfldGetDirection())){ // direction changed
        if (PfldState.sub.state == PFLD_STATE_RUNNING) {
          PfldState.sub.dir = lin_cmd;
          PfldState.sub.dir_change = ON;
          PfldMotorBrake();
        }
        else {
          PfldMotorStart( lin_cmd);
        }
      }
      else{
        //uint16 pfld_curr_raw;	   // defined at the begining of pfcyclic fn
        boolean now_at_block = FALSE;

        if( PfldState.sub.state == PFLD_STATE_BRAKING){
          if(PFLD_OVERCURRENT()) {
			aPfldSubErrorState[PfldGetDirection()==PFLD_DIR_DRIVE ? PFLD_ERRIDX_OVERCURRENT_DRIVE : PFLD_ERRIDX_OVERCURRENT_PARK] = ERROR_STATE_ERROR;
            PfldState.sub.state = PFLD_STATE_WAITING;
		
			SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(HBDSTAT))[0] =  u8HBOutputState|0x03;	
			SPIMgr_eRegWrite(HBDSTAT); 		    
          }
          else if( (++CtrlCtr) >= PFLD_BRAKE_TIME_MS/PFLD_CYCLE_TIME_MS){
            aPfldSubErrorState[PfldGetDirection()==PFLD_DIR_DRIVE ? PFLD_ERRIDX_OVERCURRENT_DRIVE : PFLD_ERRIDX_OVERCURRENT_PARK] = ERROR_STATE_OK;
            if(PfldState.sub.dir_change == ON){
              PfldMotorStart( lin_cmd);
              PfldState.sub.dir_change = OFF;
            } 
            else {
              PfldState.sub.state = PFLD_STATE_WAITING;
            }
          }
        }
        else{          
          // Overcurrent and block detection
          //if(PFLD_CM_RAW_pfld(PfldGetDirection()==PFLD_DIR_DRIVE ? PFLD_DRIVE : PFLD_PARK, &pfld_curr_raw)==E_OK){
		  if(PFLD_CM_RAW_pfld(PfldGetDirection(), &pfld_curr_raw)==E_OK){
            if(InRushCurrentCtr <= PFLD_PEAK_TIME_MS/PFLD_CYCLE_TIME_MS) {
              uint16 pfld_curr_peak_save;
            
              // disable sampletimer-ISR during copying 16-Bit value
              //PFLD_SUSPEND_ISR();
			  SysTick_vStop();
              pfld_curr_peak_save = pfld_curr_peak;
              //PFLD_RESUME_ISR();
			  SysTick_vStart();
              //if( (InRushCurrentCtr > 0)|| (pfld_curr_peak_save > PFLD_ICM_AMPS_TO_LSBS( PFLD_CURR_RUN_MIN_AMPS))){
			  if( (InRushCurrentCtr > 0)|| ( pfld_curr_peak > 400)){
                  Ubatt = VmonGetUbatRaw();
                  InRushCurrentCtr++;
                 if(InRushCurrentCtr >= PFLD_PEAK_TIME_MS/PFLD_CYCLE_TIME_MS) {
                  //PFLD_TIM_IT_CONFIG(PFLD_TIM_IT, DISABLE); // disable timer interrupt
                  	SysTick_vStop();
				  if( Ubatt < UBAT_MEAS_VOLTS_TO_LSBS(9.5)){
                    pfld_curr_block = (pfld_curr_peak*3)>>2;                // Ib=75%(Ip) if Ub<9,5V 
                  } 
                  else if( Ubatt < UBAT_MEAS_VOLTS_TO_LSBS(10.5)){
                    pfld_curr_block = (pfld_curr_peak*18)/25;               // Ib=72%(Ip) if Ub<10,5V
                  }  
                  else if( Ubatt < UBAT_MEAS_VOLTS_TO_LSBS(11.5)){
                    pfld_curr_block = (pfld_curr_peak*7)/10;                // Ib=70%(Ip) if Ub<11,5V
                  }
                  else {
                    pfld_curr_block = (pfld_curr_peak<<1)/3;                // Ib=67%(Ip) if Ub>=11,5V
                  }
                }
              }
            } 
            else {
              if( pfld_curr_raw > pfld_curr_block){
                now_at_block = TRUE;
              }
            }
            if( pfld_curr_raw > PFLD_ICM_AMPS_TO_LSBS( PFLD_CURR_MAX_AMPS)){
              PfldOvercurrentCtr++; // Count is incremented if valid result is available and I > max
            }
            else{
              PfldOvercurrentCtr = 0; // Reset counter, if current is below limit
            }
            
            if( PFLD_OVERCURRENT() || // Overcurrent flag L99MM70
               (PfldOvercurrentCtr > (PFLD_CURR_MAX_TIME_MS/PFLD_CYCLE_TIME_MS))){  // Overcurrent on highside
                aPfldSubErrorState[PfldGetDirection()==PFLD_DIR_DRIVE ? PFLD_ERRIDX_OVERCURRENT_DRIVE : PFLD_ERRIDX_OVERCURRENT_PARK] = ERROR_STATE_ERROR;
                PfldMotorBrake();
				
            }
          }
          else{
            now_at_block = TRUE; // Fail safe - if no current measured, assume blockcurrent
          }
          
          // Open Load Detection
         if( !PFLD_OPEN_LOAD(PfldGetDirection())){
              PfldOpenLoad = FALSE; // Clear if not all open-load flags are set         
              // Open Load is detected only if both open load flags were always set (Undercurrent due to Short to +Ub or GND is not detected as open-load failure!)
           	   
		      aPfldSubErrorState[PFLD_ERRIDX_OPENLOAD] = ERROR_STATE_OK;
          }
          
          if( PfldState.sub.state == PFLD_STATE_RUNNING){
            if( now_at_block == TRUE){
              PfldState.sub.state = PFLD_STATE_BLOCKING;
              CtrlCtr = 0;
            }
 //#if _EMC_TEST==OFF // Timeouts are not checked with EMC-Tests
            else{
             // typeTimerValue MaxOnTimeNow = GetMaxOnTime(); // Get timeout based on actual voltage
			  typeTimerValue MaxOnTimeNow = GetMaxOnTime(); // Get timeout based on actual voltage
              if( MaxOnTimeNow > TimeoutTicks){ // take new timeout only if it is longer
                TimeoutTicks = MaxOnTimeNow;
              }
              if( (++RunCtr) > TimeoutTicks){
                // Open Load is detected only if both open load flags were always set (Undercurrent due to interrupted wire)
                if( PfldOpenLoad){
                  aPfldSubErrorState[PFLD_ERRIDX_OPENLOAD] = ERROR_STATE_ERROR;
                }
                PfldMotorBrake();
              }
            }
//#else
//# warning EMC test mode is active!
//#endif            
          }
          else if( PfldState.sub.state == PFLD_STATE_BLOCKING){
            if( now_at_block == FALSE){
              PfldState.sub.state = PFLD_STATE_RUNNING;
            }
            else {
              typeTimerValue BlockTime =  GetBlockTime() ;  // Get timeout based on actual voltage             
			  if( (++CtrlCtr) >= BlockTime){
              PfldMotorBrake();
			  
              }
            }
          }
        }
      }
    }
  }
  ErrorArrayUpdateResult( aPfldSubErrorState, PFLD_NUM_SUB_ERRORS, LIN_ERR_IDX_PFLD);
  //SPIMgr_eRegRead(HBDSTAT);
}
/**
  * @brief  This function    
  * @param  None
  * @retval None
*/
void PfldMotorBrake(void)
{
   PfldState.sub.state = PFLD_STATE_BRAKING;
   CtrlCtr = 0;
   //PfldGetDirection()==PFLD_DIR_DRIVE? PFLD_BRAKE_DRIVE() : PFLD_BRAKE_PARK() ;
   if(PfldGetDirection() == PFLD_DIR_DRIVE )
	 PFLD_BRAKE_DRIVE();
   else	if(PfldGetDirection() == PFLD_DIR_PARK)
	 PFLD_BRAKE_PARK();
}

/**
  * @brief  This function performs all required actions to start the powerfold motor.  
  * @param  uint8 direction: direction for movement: PFLD_DIR_DRIVE or PFLD_DIR_PARK
  * @retval None
*/
static void PfldMotorStart( uint8 direction){
  uint16 pfld_curr_raw;
  PfldState.sub.dir = direction;
 
  TimeoutTicks = 0;
  PfldOvercurrentCtr = 0;
  MaxontimeCtr=0;
	   
  RunCtr = 0;
  PfldOpenLoad = TRUE; // Set at start. If one of the open-load flags is cleared at least once then we have no open load error
  if( (PfldState.sub.dir == PFLD_DIR_DRIVE) && (VmonGetUbatRaw() < UBAT_MEAS_VOLTS_TO_LSBS(PFLD_REACTIVATE_SET)))
	{
		PfldState.sub.repeat = ON;
	} 
  else 
	{
		PfldState.sub.repeat = OFF;
	}
 
  InRushCurrentCtr = 0;
  pfld_curr_peak = 0;
  if (direction==PFLD_DIR_DRIVE)
	{
		PfldState.sub.state = PFLD_STATE_RUNNING;
	 	PFLD_MOVE_DRIVE();
	 	PFLD_CM_RAW_pfld(PFLD_DIR_DRIVE,&pfld_curr_raw);
	}
  else if (direction==PFLD_DIR_PARK)
	{
		PfldState.sub.state = PFLD_STATE_RUNNING;
		PFLD_MOVE_PARK();	
		PFLD_CM_RAW_pfld(PFLD_DIR_PARK, &pfld_curr_raw);
	}

	 
  SysTick_vStart();
  // set timer, clear pending interrupt and enable interrupt
  /*  NextCompare = TimerGetUs() + PFLD_CURR_SAMPLING_US;
  PFLD_TIM_SET_COMPARE( NextCompare);
  PFLD_TIM_IT_CLEAR(PFLD_TIM_IT);    
  PFLD_TIM_IT_CONFIG(PFLD_TIM_IT, ENABLE); */
}
/**
  * @brief    
  * @param  None
  * @retval static typeTimerValue
*/
static typeTimerValue GetMaxOnTime( void){
  typeTimerValue t_max;
  uint16 Ubat = VmonGetUbatRaw();
  if( Ubat < UBAT_MEAS_VOLTS_TO_LSBS(9.5)){
    //t_max = 16000/PFLD_CYCLE_TIME_MS;
	t_max = 10000/PFLD_CYCLE_TIME_MS;	//BR changed the timing to match G2x
  }
  else if( Ubat <= UBAT_MEAS_VOLTS_TO_LSBS(10.5)){
   // t_max = 14000/PFLD_CYCLE_TIME_MS;
    t_max = 9000/PFLD_CYCLE_TIME_MS;
  }
  else if( Ubat <= UBAT_MEAS_VOLTS_TO_LSBS(11.5)){
   // t_max = 12000/PFLD_CYCLE_TIME_MS;
    t_max = 7500/PFLD_CYCLE_TIME_MS;
  }
  else if( Ubat <= UBAT_MEAS_VOLTS_TO_LSBS(12.5)){
    //t_max = 11000/PFLD_CYCLE_TIME_MS;
	 t_max = 7000/PFLD_CYCLE_TIME_MS;
  }
  else if( Ubat <= UBAT_MEAS_VOLTS_TO_LSBS(13.5)){
   // t_max = 10000/PFLD_CYCLE_TIME_MS;
    t_max = 6000/PFLD_CYCLE_TIME_MS;
  }
  else if( Ubat <= UBAT_MEAS_VOLTS_TO_LSBS(14.5)){
    //t_max = 9000/PFLD_CYCLE_TIME_MS;
	 t_max = 5500/PFLD_CYCLE_TIME_MS;
  }
  else{  // > 14.5V
    //t_max = 8000/PFLD_CYCLE_TIME_MS;
	 t_max = 5000/PFLD_CYCLE_TIME_MS;
  }
  return t_max;
}
/**
  * @brief    
  * @param  None
  * @retval static typeTimerValue
*/
static typeTimerValue GetBlockTime( void){
  typeTimerValue t_block;
  uint16 Ubat = VmonGetUbatRaw();
  if( Ubat <= UBAT_MEAS_VOLTS_TO_LSBS(11.5)){
    t_block = 300/PFLD_CYCLE_TIME_MS;
  }
  else{  // > 11,5V
    t_block = 260/PFLD_CYCLE_TIME_MS;
  }
  return t_block;
}
/**
  * @brief    
  * @param  None
  * @retval None
*/
void PfldTimerIsr( void){
  uint16 pfld_curr_raw;
  PFLD_CM_RAW_pfld( PfldGetDirection() , &pfld_curr_raw);
  if( pfld_curr_raw > pfld_curr_peak) {
       pfld_curr_peak = pfld_curr_raw;
  }
}

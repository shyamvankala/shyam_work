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
 *  @file      Htr.c
 *
 *  @desc      The following file is to implement heater functionality 
 *
 *  @document  Integrated_LINBus_Software_Design.docx
 *
 * =============================================================================
 *  Revision History
 *  ===============
 *   Date    Initials   Version   	Change
 * 07-01-17	   APR		 0.0.1    	Initial version
 
 * =============================================================================
 */			
 
#include "stdint.h"
#include "ZMDCM0.h"
#include "SPI.h"
#include "SBC.h"
#include "SBC_cfg.h"
#include "HS_cfg.h"
#include "LinAppl.h"
#include "HS.h"
#include "LINCom.h"
#include "SBC_cfg.h"
#include "LINCom_cfg.h"
#include "Applinit.h"
#include "Vmon.h"
#include "htr.h"
#include "TMR32_Cfg.h"
#include "Timer.h"
#include "TMR32.h"
#include "params.h"
#include "pfld.h"
#include "mmot.h"
#include "Error.h"
#include "Hardware.h"

// assignment of error-index
#define HTR_ERRIDX_OC      0
#define HTR_ERRIDX_OPNLD   1
#define HTR_NUM_SUB_ERRORS 2


/* Global variables #############################################################################################################*/
uint8 HtrState;
uint32_t sum_ui;	

const uint32 aSumUiLimits[4] = {
  0.25 * HTR_NOMINAL_POWER_W * (1000/HTR_CYCLE_TIME_MS) * 1000000./(UBAT_MEAS_MV_PER_LSB * HTR_I_CM_MA_FAKTOR) ,
  0.50 * HTR_NOMINAL_POWER_W * (1000/HTR_CYCLE_TIME_MS) * 1000000./(UBAT_MEAS_MV_PER_LSB * HTR_I_CM_MA_FAKTOR)  ,
  0.75 * HTR_NOMINAL_POWER_W * (1000/HTR_CYCLE_TIME_MS) * 1000000./(UBAT_MEAS_MV_PER_LSB * HTR_I_CM_MA_FAKTOR)  ,
  1.00 * HTR_NOMINAL_POWER_W * (1000/HTR_CYCLE_TIME_MS) * 1000000./(UBAT_MEAS_MV_PER_LSB * HTR_I_CM_MA_FAKTOR)  
};
static typeErrorState        aHtrSubErrorState[HTR_NUM_SUB_ERRORS]; // Array with sub-error state variables
static typeDebounceState     aHtrDebounceState[HTR_NUM_SUB_ERRORS]; // State variables for debouncing
static typeDebounceCnt       aHtrDebounceCnt[HTR_NUM_SUB_ERRORS];   // Counter variables for debouncing
static const typeDebounceCfg aHtrDebounceCfg[HTR_NUM_SUB_ERRORS] = { // Configuration for debouncing
// State                              | Timer/Counter                      | Set delay ticks                        | Reset delay ticks                      | Action                   | Action parameter
{&aHtrDebounceState[HTR_ERRIDX_OC]    , &aHtrDebounceCnt[HTR_ERRIDX_OC]    , HTR_CURR_MAX_TIME_MS/HTR_CYCLE_TIME_MS , /* not specified */0/HTR_CYCLE_TIME_MS , DEBOUNCE_ACTION_SUB_ERROR, &aHtrSubErrorState[HTR_ERRIDX_OC]},
{&aHtrDebounceState[HTR_ERRIDX_OPNLD] , &aHtrDebounceCnt[HTR_ERRIDX_OPNLD] , HTR_CURR_MIN_TIME_MS/HTR_CYCLE_TIME_MS , HTR_CURR_MIN_TIME_MS/HTR_CYCLE_TIME_MS , DEBOUNCE_ACTION_SUB_ERROR, &aHtrSubErrorState[HTR_ERRIDX_OPNLD]},
};
/* Local functions ##############################################################################################################*/
static boolean IsHeatingRequested( void);

/* Local variables ##############################################################################################################*/
static typeTimerValue t_period_start;

//static uint8_t *HSDCTRL_reg,*HSDSTAT_reg;
static uint8_t HTR_OPEN_LOAD();
// extern  volatile eHS_ShortCircuitType eLoadStatus;
 //extern volatile uint8_t u8ErrorCnt;
volatile eHS_ShortCircuitType HTR_LoadStatus;
volatile uint8_t HTR_u8ErrorCnt;
   uint8_t HTR_OpenLoad ; 
 uint8_t HTR_OC_Flag;

/**
  * @brief  Initializes all heater variables to reset state.
  * @param  None
  * @retval None
*/
void HtrInit( void){
  HtrState = HTR_STATE_IDLE;
  DebounceInit( aHtrDebounceCfg, HTR_NUM_SUB_ERRORS);
//  ErrorArrayInit( aHtrSubErrorState, HTR_NUM_SUB_ERRORS);
  	HTR_LoadStatus= eHS_Normal;
	HTR_OpenLoad =FALSE;
}

/**
  * @brief  This function must be called cyclically every HTR_CYCLE_TIME_MS.
            Updates application (checks inputs and updates statemachine and outputs) and diagnostic functions for heater.
  * @param  None
  * @retval None
*/
//TODO: Shyam should provide proper comments for each variable and values he used
void HtrCyclic( void)
{

  volatile	static	 uint16_t htr_curr_raw ;
  static uint8_t  HTR_cntr ,HTR_OC_debounce_cntr;
  
  static	 uint16_t htr_curr_raw_save ;
  switch( HtrState)
  {
  	case HTR_STATE_ON_ACTIVE:
  		{
  			    
				 boolean off_request = IsHeatingRequested() == FALSE;
			 	 if( HTR_CM_RAW( &htr_curr_raw) == E_OK)	{
				 
				 	//Cheking for sufficint current at htr terminal
					if ( htr_curr_raw >HTR_CURR_MIN_AMPS )
					{
						htr_curr_raw_save=htr_curr_raw;
					
					}	  
						if ((!HTR_OC_Flag)&& (htr_curr_raw <300) )		 
						 {
								if (HTR_cntr++ >= (HTR_OPENLoad_TIME_MS/TIMER_CYLE_MS))
								{	
									 HTR_OpenLoad = TRUE ; 

								}
						}
						else 
						{	
						HTR_cntr=0;
						HTR_OpenLoad = FALSE ;
						HTR_OUT(1);
						}

				       if( off_request == FALSE){
				  		 if( TimerGetDiffTicks( t_period_start) >= (HTR_PWM_PERIOD_MS/TIMER_CYLE_MS)){ // Start new cycle
			      			    t_period_start += HTR_PWM_PERIOD_MS/TIMER_CYLE_MS;
			       			    sum_ui = 0;
								 if (HTR_OpenLoad == FALSE )
							 {
								HTR_OUT(1);
							}		
			  			}
		       		   else{
		         			 sum_ui += (uint32)htr_curr_raw_save * (uint32)VmonGetUbatRaw();
		 				         if( sum_ui >= aSumUiLimits[l_u8_rd_BRC_CTR_HT_EXMI()-1]){
		   	 			      	 off_request = TRUE;		          		 		
								}
				 	     	 }	    	 
			               }		
				         }  
					 else{       // No current measurement -> abort heating for this period
		          		off_request = TRUE;
		     		    }	
		 	 	if( off_request){
		  	 		HTR_OUT(0);
		  	 		HtrState = HTR_STATE_ON_PASSIVE;
		   	 	}
								
  		 	 	break; }
   		case HTR_STATE_ON_PASSIVE:
    		if( TimerGetDiffTicks( t_period_start) >= (HTR_PWM_PERIOD_MS/TIMER_CYLE_MS)){ // New cycle
      			if( IsHeatingRequested()){
       				 HtrState = HTR_STATE_ON_ACTIVE;
					 if (HTR_OpenLoad == FALSE )
					 {
						HTR_OUT(1);
					}
			
        			 sum_ui = 0;
        			 t_period_start += HTR_PWM_PERIOD_MS/TIMER_CYLE_MS;
      			}
      			else{
        			HtrState = HTR_STATE_IDLE;
      			}
   			}
    		break;	   

  		case HTR_STATE_IDLE:
  			default:
    			if( IsHeatingRequested()){ // All conditions for heating fulfilled
      				HtrState = HTR_STATE_ON_ACTIVE;
      				t_period_start = TimerGet();
	 			 	HTR_OUT(1);
      				sum_ui = 0;
     		    } 
					   
   			break;
 	 }
			  if ( (HtrState==HTR_STATE_ON_PASSIVE)|| (HtrState==HTR_STATE_ON_ACTIVE))
			  {
	 			if (HTR_OpenLoad == TRUE ){
						 HTR_OUT(0);
					 HTR_LoadStatus = HS_eGetHSError(eHS1);
					if (HTR_LoadStatus == eHS_OpenLoad)
						{
							HTR_u8ErrorCnt +=2;								
      							sum_ui = 0;
						}

					else {
						   	HTR_u8ErrorCnt=0;
							HTR_OpenLoad = FALSE;
						}
						}
					
						} 
					 else if (!IsHeatingRequested())   {	HTR_OUT(0);
					HTR_LoadStatus = HS_eGetHSError(eHS1);
					if (HTR_LoadStatus == eHS_OpenLoad)
						{
							HTR_u8ErrorCnt +=2;								
      							sum_ui = 0;
							HTR_OpenLoad = TRUE;
						}

					else {
						   	HTR_u8ErrorCnt=0;
							HTR_OpenLoad = FALSE;
						} 

					 } 

				
	  			if (	DebounceUpdate(
        				&(aHtrDebounceCfg[HTR_ERRIDX_OC]),                       // Pointer to configuration
        				HTR_OC_Flag,                                       // Condition for immediate detection of overcurrent
        				HTR_OC_Flag &&(htr_curr_raw < HTR_CURR_MIN_AMPS), // Condition for delayed detection of overcurrent
        				FALSE,                                                   // Condition for immediate reset of failure (not used)
        				(!HTR_OC_Flag) &&(htr_curr_raw >HTR_CURR_MIN_AMPS)))   // Condition for delayed reset of overcurrent
						{
							if (HTR_OC_debounce_cntr++ >= (HTR_OC_debounce_TIME_MS/TIMER_CYLE_MS))	
							{ 
       				 			SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(HSDSTAT))[0] = (HTR_OC_Flag& 0xff);
								SPIMgr_eRegWrite(HSDSTAT);     
      						} 
							
						}	
				 		
						 else 	{ HTR_OC_debounce_cntr =0; }
						

											   
				DebounceUpdate(
        		&(aHtrDebounceCfg[HTR_ERRIDX_OPNLD]),                    // Pointer to configuration
        		FALSE,                                                   // Condition for immediate detection of failure (not used)
        		HTR_OPEN_LOAD()&&HTR_OpenLoad,                                         // Condition for delayed detection of open load
        		FALSE,                                                   // Condition for immediate reset of failure (not used)
        		(!HTR_OPEN_LOAD())||(!HTR_OpenLoad));                                       // Condition for delayed reset of open load (not used)

			  
      			ErrorArrayUpdateResult( aHtrSubErrorState, HTR_NUM_SUB_ERRORS, LIN_ERR_IDX_HEATER);
  
	 	  
}
	

/**
  * @brief  This function is to check all inputs and returns TRUE if heating is requested, otherwise FALSE
  * @param  None
  * @retval boolean : TRUE -> heating is requested
  *                   FALSE -> no heating requested
*/
/* Local functions ##############################################################################################################*/
static boolean IsHeatingRequested( void){
   
  return !VmonGetFailureHtr() && 
        // !TmonGetFailure() &&
         !MmotIsActive() && 
         !PfldIsActive() && 
         (l_u8_rd_BRC_CTR_EXMI_LIN() != LIN_CTRL_SELECT_ASP_SNV) &&
         (l_u8_rd_BRC_CTR_HT_EXMI() >= LIN_CTRL_HEATER_25) &&
         (l_u8_rd_BRC_CTR_HT_EXMI() <= LIN_CTRL_HEATER_100);
}


static uint8_t HTR_OPEN_LOAD()
{

	static uint8_t HTR_openLoad_flag;
  	HTR_openLoad_flag=ERROR_STATE_OK;
	 if (HTR_LoadStatus == eHS_OpenLoad)
						{
						//	HTR_u8ErrorCnt +=2;
						HTR_openLoad_flag =ERROR_STATE_ERROR;
							
						}
					else {
						  // 	HTR_u8ErrorCnt=0;
						 	HTR_openLoad_flag=  ERROR_STATE_OK ;
						}

		if ((l_u8_rd_BRC_CTR_HT_EXMI() == LIN_CTRL_HEATER_100)&& (HTR_OpenLoad == TRUE) )
			{
			HTR_openLoad_flag =ERROR_STATE_ERROR;
			}
		else if ((l_u8_rd_BRC_CTR_HT_EXMI() == LIN_CTRL_HEATER_100)&& (HTR_OpenLoad == FALSE) )
		{
			 HTR_openLoad_flag=  ERROR_STATE_OK ;
		 }


			return (HTR_openLoad_flag);
}

/*-- End of File --*/


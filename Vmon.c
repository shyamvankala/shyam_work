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
 *  @file      Vmon.c
 *
 *  @desc      The following file is to implement voltage monitoring.
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
/* Include-Files ################################################################################################################*/

#include "ADC_cfg.h"
#include "std_init.h"
#include "ZMDCM0.h"
#include "SPI.h"
#include "ADC.h"
#include "TMR32.h"
#include "SBC_cfg.h"
#include "SBC.h"
#include "SPIMgr.h"
#include "HB.h"
#include "Vmon.h"
#include "Debounce.h"
#include "Params.h"
#include "LinAppl.h"
#include "Applinit.h"

/* Defines ######################################################################################################################*/
#define VMON_UV_ZMDI_SET_DLY_MS VMON_CYCLE_TIME_MS // delay only by one cycle (10ms)
// assignment of suberror-index
#define VMON_IDX_UV      0
#define VMON_IDX_OV      1
#define VMON_IDX_UV_HTR  2
#define VMON_NUM_MONI    3
//#define ADC_CH_UBAT      eNdxAIN_VDDE		   //for ADC VOLTAGE 
 
/* Type-definitions #############################################################################################################*/
/* Declarations #################################################################################################################*/
/* Local functions ##############################################################################################################*/
uint8_t VMON_OVERVOLT(void);
uint8_t VMON_UNDERVOLT(void);
/* Global variables #############################################################################################################*/
uint8_t bVmonFailure, bVmonFailureHtr;
uint16_t UbatRaw ,UHc2Gnd , voltage ;
  
/* Local variables ##############################################################################################################*/
static uint8_t ZMDIUvCtr;
static typeDebounceState     aVmonDebounceState[VMON_NUM_MONI]; // State variables for debouncing
static typeDebounceCnt       aVmonDebounceCnt[VMON_NUM_MONI];
static const typeDebounceCfg aVmonDebounceCfg[VMON_NUM_MONI] = { // Configuration for debouncing
// State                              | Timer/Counter                      | Set delay ticks                           | Reset delay ticks                           | Action                    | Action parameter
{&aVmonDebounceState[VMON_IDX_UV]     , &aVmonDebounceCnt[VMON_IDX_UV]     , VMON_UV_SET_DLY_MS/VMON_CYCLE_TIME_MS     , VMON_UV_RESET_DLY_MS/VMON_CYCLE_TIME_MS     , DEBOUNCE_ACTION_LIN_ERROR, (void*)LIN_ERR_IDX_UNDERVOLTAGE},
{&aVmonDebounceState[VMON_IDX_OV]     , &aVmonDebounceCnt[VMON_IDX_OV]     , VMON_OV_SET_DLY_MS/VMON_CYCLE_TIME_MS     , VMON_OV_RESET_DLY_MS/VMON_CYCLE_TIME_MS     , DEBOUNCE_ACTION_LIN_ERROR, (void*)LIN_ERR_IDX_OVERVOLTAGE},
{&aVmonDebounceState[VMON_IDX_UV_HTR] , &aVmonDebounceCnt[VMON_IDX_UV_HTR] , VMON_UV_HTR_SET_DLY_MS/VMON_CYCLE_TIME_MS , VMON_UV_HTR_RESET_DLY_MS/VMON_CYCLE_TIME_MS , DEBOUNCE_ACTION_LIN_ERROR, (void*)LIN_ERR_IDX_UV_HEATER},
};		 

/**
  * @brief  Initializes the variables used for voltage monitoring
  * @param  None
  * @retval None
*/
void VmonInit( void){
  UbatRaw = UBAT_MEAS_VOLTS_TO_LSBS(12.7); // Initialize with uncritical value
  DebounceInit( aVmonDebounceCfg, VMON_NUM_MONI);
  bVmonFailure = FALSE;
  ZMDIUvCtr = 0;
}
/**
  * @brief  Reads ubat related voltages and updates the states for over- and undervoltage detection
  * @param  None
  * @retval None
*/
void VmonCyclic( void){

 boolean ZMDIUvQualified = FALSE;

 UbatRaw = ( AdcRead( ADC_CH_UBAT)+ VOLTAGE_OFFSET( AdcRead( ADC_CH_UBAT))) ;

  //Checking Undervoltage condition based on ADC Raw values
  
  if(UbatRaw <= UBAT_MEAS_VOLTS_TO_LSBS(VMON_UV_SET_DELAYED))
	{
		volatile uint8_t* VMON_UV_stat;
		SPIMgr_eRegRead(RSTSTAT);
		VMON_UV_stat = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(RSTSTAT));
	
		SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(RSTSTAT))[0] = *VMON_UV_stat | 0x10;	
		SPIMgr_eRegWrite(RSTSTAT); 
			
	}
		 //Checking Overvoltage condition based on ADC Raw values

	    else if	(UbatRaw>UBAT_MEAS_VOLTS_TO_LSBS(VMON_OV_SET_IMMEDIATE))
		{
		volatile uint8_t* VMON_OV_stat;
		SPIMgr_eRegRead(RSTSTAT);
		VMON_OV_stat = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(RSTSTAT));
	
		SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(RSTSTAT))[0] = *VMON_OV_stat | 0x20;	
		SPIMgr_eRegWrite(RSTSTAT); 
		}

		  //Over voltage detection based on ZAMC 4200 Status Register Value 
  	if( VMON_OVERVOLT()){
  		  UbatRaw = UBAT_MEAS_VOLTS_TO_LSBS(20); // If ZMDI has overvoltage condition, Ubat measurement is switched off -> set value at top level
  		}
	 //Under voltage detection based on ZAMC 4200 Status Register Value
		  if( VMON_UNDERVOLT()){
		    if( ZMDIUvCtr < VMON_UV_ZMDI_SET_DLY_MS/VMON_CYCLE_TIME_MS){
		      ZMDIUvCtr++;
		    }
		    else{
		      ZMDIUvQualified = TRUE;
		    }
		  }
		  else{
		    ZMDIUvCtr = 0;
		  }	

   // Initialize VmonFailure-Flags
  bVmonFailure = bVmonFailureHtr = 0;

   // Calculation of undervoltage condition
  if( DebounceUpdate(
    &(aVmonDebounceCfg[VMON_IDX_UV]),                        // Pointer to configuration	
    /*ZMDIUvQualified*/ FALSE,                                          // Condition for immediate detection of undervoltage
    (UbatRaw<(UBAT_MEAS_VOLTS_TO_LSBS(VMON_UV_SET_DELAYED))),  // Condition for delayed detection of undervoltage
    FALSE,                                                   // Condition for immediate reset of failure (not used)
    UbatRaw>UBAT_MEAS_VOLTS_TO_LSBS(VMON_UV_RESET_DELAYED))){ // Condition for delayed reset of undervoltage
    bVmonFailure = bVmonFailureHtr = TRUE;
  }

  // Calculation of overvoltage condition
  if( DebounceUpdate(
    &(aVmonDebounceCfg[VMON_IDX_OV]),                        // Pointer to configuration
    (UbatRaw>UBAT_MEAS_VOLTS_TO_LSBS(VMON_OV_SET_IMMEDIATE)) ||
    VMON_OVERVOLT(),                            // Condition for immediate detection of overvoltage
    UbatRaw>UBAT_MEAS_VOLTS_TO_LSBS(VMON_OV_SET_DELAYED),    // Condition for delayed detection of overvoltage
    FALSE,                                                   // Condition for immediate reset of overvoltage (not used)
    UbatRaw<UBAT_MEAS_VOLTS_TO_LSBS(VMON_OV_RESET_DELAYED))){ // Condition for delayed reset of overvoltage
    bVmonFailure = bVmonFailureHtr = TRUE;
  }

  // Calculation of undervoltage condition heater
  if( DebounceUpdate(
    &(aVmonDebounceCfg[VMON_IDX_UV_HTR]),                       // Pointer to configuration
 	 ZMDIUvQualified,                                             // Condition for immediate detection of undervoltage
    (UbatRaw<UBAT_MEAS_VOLTS_TO_LSBS(VMON_UV_HTR_SET_DELAYED)), // Condition for delayed detection of undervoltage
    FALSE,                                                      // Condition for immediate reset of overvoltage (not used)
    UbatRaw>UBAT_MEAS_VOLTS_TO_LSBS(VMON_UV_HTR_RESET_DELAYED))){// Condition for delayed reset of undervoltage
    bVmonFailureHtr = TRUE;
  }
}
/**
  * @brief  Function returns the Vmon debounce state in debug mode
  * @param  None
  * @retval aVmonDebounceState	- pointer to the debounce state of VMON
*/

#if _DEBUG==ON // for debugging only  
const typeDebounceState* VmonGetDebounceStates( void){
  return aVmonDebounceState;
}
#endif
															
/**
  * @brief  Function checks the VMON OverVOLTAGE flag status on RSTSTAT register
  * @param  None
  * @retval 0	- if OV flag is not set
  *			1   - if OV falg is set
*/
uint8_t VMON_OVERVOLT(void)
{
	volatile uint8_t* VMON_OV_stat;
	//SPI procedure call for reading RSTSTAT Register
	SPIMgr_eRegRead(RSTSTAT);
	VMON_OV_stat = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(RSTSTAT));
	//Checking the OV bit set value
	if((*VMON_OV_stat & 0x20) == 0X20) {
		
		return 1;		
		}
	else{
		 
		return 0;

	 }
}

/**
  * @brief  Function checks the VMON Undercurrent flag status on RSTSTAT register
  * @param  None
  * @retval 0	- if UV flag is not set
  *			1   - if UV falg is set
*/
uint8_t VMON_UNDERVOLT(void)
{
	volatile uint8_t* VMON_UV_stat;
	//SPI procedure call for reading RSTSTAT Register
	SPIMgr_eRegRead(RSTSTAT);
	//Checking the UV bit set value
	VMON_UV_stat = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(RSTSTAT));
	if((*VMON_UV_stat & 0x10) == 0x10) {
		
		return 1;		
	}
	else{
		 
		return 0;

	}
}


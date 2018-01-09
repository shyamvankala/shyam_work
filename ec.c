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
 *  @file      Ec.c
 *
 *  @desc      Sourcefile for Ec (Electrochromic glass) functions (control and diagnostics) 
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
#define _EC_C_

/* Include-Files ################################################################################################################*/
#include "ZMDCM0.h"
#include "stdint.h"
#include "std_init.h"
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
#include "Ec.h"
#include "Adc.h"
#include "debounce.h"
#include "LinAppl.h"

// assignment of error-index
#define EC_ERRIDX_HS_OC           0 // overcurrent on highside
#define EC_ERRIDX_LS_OC           1 // overcurrent on lowside
#define EC_ERRIDX_ECV_MONI_FAIL   2 // L99MM70 ECV monitoring failure
#define EC_ERRIDX_ECV_OV_OPNLD    3 // overvoltage or open load detected by ADC measurement MCU
#define EC_NUM_SUB_ERRORS         4
#define EC_VOLTAGE_STATUS_LOW()   StatusECVolt.UV_Status//0//gL99StatReg3.sig.ECV_MONI_LOW
#define EC_VOLTAGE_STATUS_HIGH()  StatusECVolt.OV_Status //0//gL99StatReg3.sig.ECV_MONI_HIGH
#define EC_OFFSET                 16
// Check range for EC-Glass activation
#define LIN_CTRL_EC_DIMMED() ((l_u8_rd_BRC_CTR_ECR_EXMI_DIP_LIN() >= 1) && (l_u8_rd_BRC_CTR_ECR_EXMI_DIP_LIN() <= 100))
#define ECM_MINVOLT 5
#define ECM_MAXVOLT 520 //513 for 100% and outpput voltage 1.202

typedef struct{
	uint8_t UV_Status;
	uint8_t OV_Status;
}EC_UV_OV_Status;

extern uint8_t bVTFailure;

static void EcStateDimmed( void);
static void EcStateClearing( void);
static uint16 DelayCnt;
volatile uint16_t ECM_volt;

volatile EC_UV_OV_Status StatusECVolt;

static typeErrorState        aEcSubErrorState[EC_NUM_SUB_ERRORS]; // Array with sub-error state variables
static typeDebounceState     aEcDebounceState[EC_NUM_SUB_ERRORS]; // State variables for debouncing
static typeDebounceCnt       aEcDebounceCnt[EC_NUM_SUB_ERRORS];   // Counter variables for debouncing
static const typeDebounceCfg aEcDebounceCfg[EC_NUM_SUB_ERRORS] = { // Configuration for debouncing
// State                                    | Timer/Counter                            | Set delay ticks                      | Reset delay ticks                    | Action                    | Action parameter
{&aEcDebounceState[EC_ERRIDX_HS_OC]         , &aEcDebounceCnt[EC_ERRIDX_HS_OC]         , EC_CURR_MAX_TIME_MS/EC_CYCLE_TIME_MS , /* no specified */0/EC_CYCLE_TIME_MS , DEBOUNCE_ACTION_SUB_ERROR, &aEcSubErrorState[EC_ERRIDX_HS_OC]},
{&aEcDebounceState[EC_ERRIDX_LS_OC]         , &aEcDebounceCnt[EC_ERRIDX_LS_OC]         , /* no specified */0/EC_CYCLE_TIME_MS , /* no specified */0/EC_CYCLE_TIME_MS , DEBOUNCE_ACTION_SUB_ERROR, &aEcSubErrorState[EC_ERRIDX_LS_OC]},
{&aEcDebounceState[EC_ERRIDX_ECV_MONI_FAIL] , &aEcDebounceCnt[EC_ERRIDX_ECV_MONI_FAIL] , EC_VMON_DET_DELAY_MS/EC_CYCLE_TIME_MS, EC_VMON_DET_DELAY_MS/EC_CYCLE_TIME_MS, DEBOUNCE_ACTION_SUB_ERROR, &aEcSubErrorState[EC_ERRIDX_ECV_MONI_FAIL]},
{&aEcDebounceState[EC_ERRIDX_ECV_OV_OPNLD]  , &aEcDebounceCnt[EC_ERRIDX_ECV_OV_OPNLD]  , EC_ADC_DEBOUNCE_MS/EC_CYCLE_TIME_MS  , EC_ADC_DEBOUNCE_MS/EC_CYCLE_TIME_MS  , DEBOUNCE_ACTION_SUB_ERROR, &aEcSubErrorState[EC_ERRIDX_ECV_OV_OPNLD]},
};

void EC_UV_OV_ADCRead(void);


/**
  * @brief  This function Initializes all variables to reset state (EC cleared)       
  * @param  None
  * @retval None
*/
void EcInit( void){
  // after reset, clear EC glass
  EcStateClearing(); 
  //Initializing debounce parameters
  DebounceInit( aEcDebounceCfg, EC_NUM_SUB_ERRORS);
  //Initializing error state parameters
  ErrorArrayInit( aEcSubErrorState, EC_NUM_SUB_ERRORS);
}
/**
  * @brief  This function Must be called cyclically every EC_CYCLE_TIME_MS.
  *         Updates application (checks inputs and updates statemachine and outputs) and diagnostic functions for electro-
  *        chrome.       
  * @param  None
  * @retval None
*/
void EcCyclic( void){
  //Reading uv & ov conditions using ADC channel
  	
		
	
  EC_UV_OV_ADCRead();
  // First check if OFF-Delay has elapsed
  if( EcState == EC_STATE_OFF_DELAY){
  	//settinf EC OFF
    EC_CONTROL_OFF();
    if( ++DelayCnt >= (EC_OVERCURR_DELAY_MS/EC_CYCLE_TIME_MS)){
      EcState = EC_STATE_OFF;
      DelayCnt = 0;
    }
  }
  else if( VTFailure() || (l_u8_rd_BRC_CTR_EXMI_LIN() == LIN_CTRL_SELECT_ASP_SNV)){ // Actual state not changed but output set floating
    EC_CONTROL_OFF();
  }
  else if( EcState == EC_STATE_OFF){
    if( LIN_CTRL_EC_DIMMED()){
	  //Setting EC in dimmed state
      EcStateDimmed();
    }
    else{ // No activation request -> discharge
  #if EC_FLOATING_TIME_MS>0
      if( ++DelayCnt >= (EC_FLOATING_TIME_MS/EC_CYCLE_TIME_MS)){
	  	//Clearing EC States      
        EcStateClearing();
      }
      else
  #endif
      {
        // !TODO switch to EC_CONTROL_OFF or stay in EC_CONTROL_OFF_DAC_MIN???
        // If changed to EC_CONTROL_OFF_DAC_MIN overcurrent-monitoring for Out10 is
        // an issue!
        EC_CONTROL_OFF();
      }
    }
  }
  else if( EcState == EC_STATE_CLEARING){
  
  /*  if( DebounceUpdate( // Check for overcurrent on ECV lowside
      &(aEcDebounceCfg[EC_ERRIDX_LS_OC]), // Pointer to configuration
      EC_LS_OVERCURRENT(),                // Condition for immediate detection of failure 
      FALSE,                              // Condition for delayed detection of failure (not used)
      FALSE,                              // Condition for immediate reset of failure (not used)
      TRUE)){                             // Condition for delayed reset of failure (always if not set)
      EC_CONTROL_OFF_DAC_MIN();
      EcState = EC_STATE_OFF_DELAY;
      DelayCnt = 0;
    }  */
  //  else {
      if( LIN_CTRL_EC_DIMMED()){
        EcStateDimmed();
      }
  #if EC_CLEARING_TIME_MS>0
      else if( ++DelayCnt >= (EC_CLEARING_TIME_MS/EC_CYCLE_TIME_MS)){
        EC_CONTROL_OFF_DAC_MIN();
        EcState = EC_STATE_OFF;
        DelayCnt = 0;
      }
  #endif
      else{
        EC_CONTROL_CLEAR();
      }
   // }
  }
  else if( EcState == EC_STATE_DIMMED){
   // uint16 ec_curr_raw;
  
   ECM_volt = AdcRead(eNdxAIN_ECM); // read EC voltage , if possible

    if( DebounceUpdate( // Check for overcurrent on EC highside switch
      &(aEcDebounceCfg[EC_ERRIDX_HS_OC]),                       // Pointer to configuration
      FALSE,                                      // Condition for immediate detection of failure
      (ECM_volt < 20 ),   // Condition for delayed detection of failure
      FALSE,                                                    // Condition for immediate reset of failure (not used)
      (ECM_volt < 20))){ // Condition for delayed reset of failure
      EC_CONTROL_OFF();
      EcState = EC_STATE_OFF_DELAY;
      DelayCnt = 0;
    }
    else{
      if( !LIN_CTRL_EC_DIMMED()){
        EcStateClearing();
      }
      else{        
        EC_CONTROL( EC_PERCENT_TO_DAC_VALUE( l_u8_rd_BRC_CTR_ECR_EXMI_DIP_LIN()));
      }
    }
   	/*  EC_UV_OV_ADCRead();
      // Calculation of EC Glass voltage monitor
      DebounceUpdate(
      &(aEcDebounceCfg[EC_ERRIDX_ECV_MONI_FAIL]),                        // Pointer to configuration
      FALSE,                                                             // Condition for immediate detection of failure (not used)
      EC_VOLTAGE_STATUS_LOW() ||
      (EC_VOLTAGE_STATUS_HIGH() && 
      (l_u8_rd_BRC_CTR_ECR_EXMI_DIP_LIN() > EC_VMON_HIGH_PERCENT_MIN)),  // Condition for delayed detection of failure
      FALSE,                                                             // Condition for immediate reset of failure (not used)
      !EC_VOLTAGE_STATUS_LOW() && 
      (!EC_VOLTAGE_STATUS_HIGH() || 
      (l_u8_rd_BRC_CTR_ECR_EXMI_DIP_LIN() < EC_VMON_HIGH_PERCENT_MIN)));*/ // Condition for delayed reset of failure
   
  }	
 /* 
  // Calculation of EC Glass ADC voltage monitoring
  DebounceUpdate(
    &(aEcDebounceCfg[EC_ERRIDX_ECV_OV_OPNLD]), // Pointer to configuration
    FALSE,                                     // Condition for immediate detection of failure (not used)
    EC_OPNLD_IN_DIGITAL(),                     // Condition for delayed detection of failure
    FALSE,                                     // Condition for immediate reset of failure (not used)
    EC_OPNLD_SWITCH_IS_ON() &&
    EC_OPNLD_IN_DIGITAL()==0);                 // Condition for delayed reset of failure
     */
  ErrorArrayUpdateResult( aEcSubErrorState, EC_NUM_SUB_ERRORS, LIN_ERR_IDX_EC_GLASS);
  
  // Activate highside switch for open-load detection always if discharging is not active
  //if( EC_OPNLD_SWITCH_IS_ON() != (EcState != EC_STATE_CLEARING)){
  // EC_OPNLD_SWITCH_PUT( EcState != EC_STATE_CLEARING);
  //} */
  //}
   }
#if _DEBUG==ON // for debugging only  
/**
  * @brief  This function Returns a pointer to the internal diagnostic information - for debugging only     
  * @param  None
  * @retval const typeErrorState*: pointer to internal errorstates.
*/
const typeErrorState* EcGetErrors( void){
  return aEcSubErrorState;
}
#endif
/**
  * @brief  This function Performs all required actions to change EC to state "dimmed".    
  * @param  None
  * @retval None
*/
static void EcStateDimmed( void){
  EC_CONTROL( EC_PERCENT_TO_DAC_VALUE( l_u8_rd_BRC_CTR_ECR_EXMI_DIP_LIN()));
  EcState = EC_STATE_DIMMED;
}
/**
  * @brief  This function Performs all required actions to change EC to state "clearing"
  * @param  None
  * @retval None
*/
static void EcStateClearing( void){
  EC_CONTROL_CLEAR();
  EcState = EC_STATE_CLEARING;
  DelayCnt = 0;
}

/**
  * @brief  This function calculates the DAC value based on output voltage range 
  * @param  None
  * @retval None
*/
uint8_t EC_PERCENT_TO_DAC_VALUE( uint8_t percent)
{
	 uint8_t u8Return = 0;
	 //Logic to obating EC output voltage in the range of 0.3 V to 1.2V
	 //TODO: This logic can be either removed or modified for various loads and specifications
	 if(percent <= 20)
	 {
	 	u8Return = ((percent)/3 + EC_OFFSET);	
	 }
	 else if ((percent > 20) && (percent <= 30) )
	 {
	 	u8Return = ((percent)/3 + EC_OFFSET+ 2 );
	 }
	 else if ((percent > 30) && (percent <= 45) )
	 {
	 	u8Return = ((percent)/3 + EC_OFFSET+ 2);
	 }
	 else if ((percent > 45) && (percent <= 60) )
	 {
	 	u8Return = ((percent)/3 + EC_OFFSET+ 4);
	 }
	 else if ((percent > 60) && (percent <= 75) )
	 {
	 	u8Return = ((percent)/3 + EC_OFFSET+ 6);
	 }
	 else if ((percent > 75) && (percent < 99) )
	 {
	 	u8Return = ((percent)/3 + EC_OFFSET+ 9);
	 }
	 else if (percent >= 99)
	 {
	 	u8Return = ((percent)/3 + EC_OFFSET+ 6);
	 }

	 return u8Return;
}
/**
  * @brief  This function check under voltage and over voltage conditions and update the status flags 
  * @param  None
  * @retval None
*/
//TODO:This can be removed when Shyam implement his algorithm
void EC_UV_OV_ADCRead(void){
		
	
		if ( (ECM_volt < ECM_MINVOLT) && (EcState != EC_STATE_OFF)){
		  StatusECVolt.OV_Status = 0;
		  StatusECVolt.UV_Status = 1;
		}
		else if (ECM_volt >= ECM_MAXVOLT)
		{
		  StatusECVolt.OV_Status = 1;
		  StatusECVolt.UV_Status = 0;
		}
		else if	 ((ECM_volt >= ECM_MINVOLT) && (ECM_volt < ECM_MAXVOLT)){
		  StatusECVolt.OV_Status = 0;
		  StatusECVolt.UV_Status = 0;
		}


}
/**
  * @brief  This function check open load and short to supply conditions and update the status flags 
  * @param  None
  * @retval uint8_t	: 1 - if OL/SS flag is set
  *					  0	- if OL/SS flag is  not set
*/
//TODO:This can be removed when Shyam implement his algorithm
uint8_t EC_OL_SS_DIAGStat(void)
{
	volatile uint8_t* ECMOSFlag_stat;
	// SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(ECMCTRL))[0] =  0x00;	
  	//SPIMgr_eRegWrite(ECMCTRL);
	 SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(ECMDIAG))[0] =  0x08;	
  	SPIMgr_eRegWrite(ECMDIAG);

	SPIMgr_eRegRead(ECMDIAG);
	ECMOSFlag_stat = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(ECMDIAG));
		if((*ECMOSFlag_stat & 0x02) == 2) 
		return 1;
	else
		return 0;
	 
}
/**
  * @brief  This function check High Side and low side overcurrent flags and also to update the status flags 
  * @param  None
  * @retval uint8_t	: 1 - if HS/LS Overcurrent flag is set
  *					  0	- if HS/LS Overcurrent flag is  not set
*/
//TODO:This can be removed when Shyam implement his algorithm
uint8_t EC_HS_LS_OVERCURRENT(void)
{
	volatile uint8_t* ECMOSFlag_stat;
	// SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(ECMCTRL))[0] =  0x00;	
  	//SPIMgr_eRegWrite(ECMCTRL);
	 SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(ECMDIAG))[0] =  0x08;	
  	SPIMgr_eRegWrite(ECMDIAG);

	SPIMgr_eRegRead(ECMDIAG);
	ECMOSFlag_stat = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(ECMDIAG));
		if((*ECMOSFlag_stat & 0x01) == 1) {
		
		return 1;		
		}
	else{
		 
		return 0;

	 }
}




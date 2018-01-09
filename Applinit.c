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
 *  @file      Applinit.c
 *
 *  @desc      The following file is to implement current monitoring and also
 			   setting haifbridges for powerfold operation
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
#include "std_init.h"
#include "ZMDCM0.h"
#include "SPI.h"
#include "ADC.h"
#include "Applinit.h"
#include "HB.h"
#include "pfld.h"



static enum{
  CURR_MONI_STATE_INIT,
  CURR_MONI_STATE_IDLE,
  CURR_MONI_STATE_SWITCH_REQUEST,
  CURR_MONI_STATE_SWITCHED
}CurrMoniState;

static uint16_t CurrMoniSwitchTimeStart;
static uint16_t CurrMoniAdcValue; 


extern uint8_t  pfld_active_flag;



/**
  * @brief  Reads and update currentmonitor	using ADC
  * @param  None
  * @retval None
  TODO:This function is currently unused. Clean it up before final release if not required
*/
void SBC_ApplUpdateCurrentmonitor( uint8_t MuxVal){

  // Measure actual voltage at CM-Pin if minimum delay has elapsed  
   
  uint8_t Channel_MUX ;
  Channel_MUX = MuxVal ;
  //  PFLD_SUSPEND_ISR();
  //  if( CurrMoniState == CURR_MONI_STATE_SWITCHED){
  //    if( TimerGetElapsedUs( CurrMoniSwitchTimeStart) > L99_CURRENT_MONITOR_SWITCH_DELAY_US)
  //     CurrMoniState = CURR_MONI_STATE_IDLE;      
  //  }
  //  }
  //  if( CurrMoniState == CURR_MONI_STATE_IDLE){
  //	curr_raw = AdcRead(MuxVal);	 
  //  return   CurrMoniAdcValue;
  //  }
  // PFLD_RESUME_ISR();
}
/**
  * @brief  This function is used by the application to get values from the currentmonitor.
  * @param  uint8 MuxVal    : requested channel for currentmonitor
  *			uint16* pCurrent: pointer to variable where to store the measured 10 bit adc value.
  * @retval Std_ReturnType : E_OK if no error, otherwise E_NOT_OK
*/
Std_ReturnType ApplReadCurrentMonitorRaw_pfld( uint8 MuxVal, uint16* pCurrent){
	  Std_ReturnType ret;
	  static uint8_t curr_raw;
	  // if( (CurrMoniState != CURR_MONI_STATE_IDLE))// || 
	  //   (MuxVal != gL99CtrlReg4.sig.CM_SELECT))
	  //{
	  //   *pCurrent = 0; // in case of error the current is set defined to zero. So no overcurrent error is detected.
	  // ret = E_NOT_OK;    
	  //}
	  //  else{ // return last measured value

	  if(MuxVal == PFLD_CM_drive)
	    //Read poerfold raw current value during drive 
      	*pCurrent = (AdcRead(PFLD_CM_drive)*1000*(110/100))*RESCODE;
	  else if(MuxVal == PFLD_CM_park)
	  	//Read poerfold raw current value during drive 
	    *pCurrent = (AdcRead(PFLD_CM_park)*1000*(110/100))*RESCODE;
	
    	ret = E_OK;    
 	  // }
  return ret;  
}
//TODO: Cleane up if unnecessary
/*
Std_ReturnType ApplReadCurrentMonitorRaw_park( uint8 MuxVal, uint16* pCurrent){
  Std_ReturnType ret;
   static uint8_t curr_raw;
 // if( (CurrMoniState != CURR_MONI_STATE_IDLE))// || 
 //   (MuxVal != gL99CtrlReg4.sig.CM_SELECT))
 //{
 //   *pCurrent = 0; // in case of error the current is set defined to zero. So no overcurrent error is detected.
   // ret = E_NOT_OK;    
  //}
//  else{ // return last measured value

   	 *pCurrent = (AdcRead(PFLD_CM_park)*1000*(110/100))*RESCODE;


    ret = E_OK;    
 // }
  return ret;  
}
*/
//TODO: Shyam has to provide the definition and comments 
Std_ReturnType ApplReadCurrentMonitorRaw( uint8 MuxVal, uint16* pCurrent){
  Std_ReturnType ret;
  static uint8_t curr_raw;
  // if( (CurrMoniState != CURR_MONI_STATE_IDLE))// || 
  //   (MuxVal != gL99CtrlReg4.sig.CM_SELECT))
  //{
  //   *pCurrent = 0; // in case of error the current is set defined to zero. So no overcurrent error is detected.
  // ret = E_NOT_OK;    
  //}
  //  else{ // return last measured value

  switch (MuxVal)
	{
    	case HTR_CM: 
		 *pCurrent =((AdcRead(MuxVal)*10)/1.3);
		  break;
		 case HC2_CM:
		 *pCurrent = (AdcRead(MuxVal)/1.7);
		  break;										
	}
    ret = E_OK;    
  // }
  return ret;  
}
/**
  * @brief  Sets the half bridges for powerfold drive operation
  * @param  None
  * @retval None
  TODO: Clean it up before final release if not required
*/
void PFLD_MOVE_DRIVE()
 {  
 	// TODO: Remove the flag if unnecessary
	pfld_active_flag=1;
	//Initialize HB driver
	//HB_vInit();
	//Enable low side of HB2
	HB_vHBControl(eHB2,eHB_LON_HOFF);
	//Enable high side of HB1
   	HB_vHBControl(eHB1,eHB_LOFF_HON);
	//Update the powerfold state machine
	PfldState.sub.state = PFLD_STATE_RUNNING;
	//	HB_vTask();
	//Update the HB registers	
	HB_vUpdateControl();	
 }
/**
  * @brief  Sets the half bridges for powerfold park operation
  * @param  None
  * @retval None
  TODO: Clean it up before final release if not required
*/
void PFLD_MOVE_PARK()
 {  
 	// TODO: Remove the flag if unnecessary
	pfld_active_flag=1;
	//Initialize HB driver
//	HB_vInit();
	//Enable low side of HB1
 	HB_vHBControl(eHB1,eHB_LON_HOFF);
	//Enable high side of HB2
   	HB_vHBControl(eHB2,eHB_LOFF_HON);
	//Update the powerfold state machine
	PfldState.sub.state = PFLD_STATE_RUNNING;
    //HB_vTask();
	//Update the HB registers
	HB_vUpdateControl();
 }
/**
  * @brief  Sets the half bridges for powerfold brake during drive operation
  * @param  None
  * @retval None
  TODO: Clean it up before final release if not required
*/
void PFLD_BRAKE_DRIVE()
 {

	//	HB_vInit();
	//Disable HB1 
	HB_vHBControl(eHB1,eHB_OFF);
	//Disable HB2
	HB_vHBControl(eHB2,eHB_OFF);
	//	HB_vHBControl(eHB2,eHB_LON_HOFF);
	//	HB_vHBControl(eHB1,eHB_LON_HOFF);
	//Update the HB registers
	HB_vUpdateControl();
	HB_vInit();
	// TODO: Remove the flag if unnecessary 
	pfld_active_flag=0;
 }
/**
  * @brief  Sets the half bridges for powerfold brake during PARK operation
  * @param  None
  * @retval None
  TODO: Clean it up before final release if not required
*/
void PFLD_BRAKE_PARK()
 {
	//	HB_vInit();
	//Disable HB1
	HB_vHBControl(eHB1,eHB_OFF);
	//Disable HB2
	HB_vHBControl(eHB2,eHB_OFF);
	//	HB_vHBControl(eHB2,eHB_LON_HOFF);
	//	HB_vHBControl(eHB1,eHB_LON_HOFF);
	//Update the HB registers
	HB_vUpdateControl();
	// TODO: Remove the flag if unnecessary
	pfld_active_flag=0;
 }

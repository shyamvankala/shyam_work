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
 *  @file      ZMDIappl.c
 *
 *  @desc      Sourcefile for ZAMC driver application specific part
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
#define _L99APPL_C_

#include "stdint.h"
#include "Pfld.h"
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
#include "params.h"
#include "Mmot.h"
#include "Ec.h"


#define MOTORS_IDLE()             HB_vHBControl(eHB1,eHB_OFF); HB_vHBControl(eHB2,eHB_OFF);HB_vHBControl(eHB3,eHB_OFF); HB_vHBControl(eHB4,eHB_OFF);


static enum{
  CURR_MONI_STATE_INIT,
  CURR_MONI_STATE_IDLE,
  CURR_MONI_STATE_SWITCH_REQUEST,
  CURR_MONI_STATE_SWITCHED
}CurrMoniState;
static uint16 CurrMoniSwitchTimeStart;
static uint16 CurrMoniAdcValue; // Last conversion result
typePfldState PfldState;
/********************************************************************************************************************************/
//TODO: The below part can be cleaned up if it is not used until final version
const struct{
  uint16 HbOnMask;
  uint8 MuxChannel;
}MmotDriveCfg[4]; /*= {
 L99_HBR_OUT_HS_MASK(MMOT_LEFT)|L99_HBR_OUT_LS_MASK(MMOT_RIGHT), MMOT_LEFT, // left
 L99_HBR_OUT_LS_MASK(MMOT_LEFT)|L99_HBR_OUT_HS_MASK(MMOT_RIGHT), MMOT_RIGHT,// right
 L99_HBR_OUT_HS_MASK(MMOT_UP)  |L99_HBR_OUT_LS_MASK(MMOT_DOWN) , MMOT_UP,   // up
 L99_HBR_OUT_LS_MASK(MMOT_UP)  |L99_HBR_OUT_HS_MASK(MMOT_DOWN) , MMOT_DOWN  // down
};*/

// braking highside
const uint16 MmotBrakeHbOnMask[2];/* = {
  L99_HBR_OUT_HS_MASK(MMOT_LEFT)|L99_HBR_OUT_HS_MASK(MMOT_RIGHT), // x
  L99_HBR_OUT_HS_MASK(MMOT_UP)  |L99_HBR_OUT_HS_MASK(MMOT_DOWN)   // y
}; */

/*
// braking lowside
const uint16 MmotBrakeHbOnMask[2] = {
  L99_HBR_OUT_LS_MASK(MMOT_LEFT)|L99_HBR_OUT_LS_MASK(MMOT_RIGHT), // x
  L99_HBR_OUT_LS_MASK(MMOT_UP)  |L99_HBR_OUT_LS_MASK(MMOT_DOWN)   // y
};*/
/********************************************************************************************************************************/
void ZMDIMmotHBConfig(void);  
void ZMDIApplInit(void);

/**
  * @brief  This function configure half bridges for glass drive and memory operations  
  * @param  None
  * @retval None
*/
//TODO: Remove Commented functions if unnecessary
void ZMDIMmotHBConfig(void){
	switch (MmotGetDirection()){
#if defined _LEFT_MIRROR
	case MMOT_DIR_UP :
		//Initialize HB module
		//HB_vInit();
      	//HB_vHBControl(eHB3,eHB_LON_HOFF);
		//Turn on the low side of half bridge driver 4
	  	HB_vHBControl(eHB4,eHB_LON_HOFF);
	 	//Turn on the high side of half bridge driver 2
	 	HB_vHBControl(eHB2,eHB_LOFF_HON);
	    //HB_vHBControl(eHB4,eHB_OFF);
	    HB_vUpdateControl();
	    //	HB_vTask();
	break;
    case MMOT_DIR_DOWN:
		//Initialize HB module
		//HB_vInit();
		//Turn on the low side of half bridge driver 2
        HB_vHBControl(eHB2,eHB_LON_HOFF);
	 	//HB_vHBControl(eHB3,eHB_LOFF_HON);
		//Turn on the high side of half bridge driver 4
	 	HB_vHBControl(eHB4,eHB_LOFF_HON);
	  	//HB_vHBControl(eHB4,eHB_OFF);
	  	HB_vUpdateControl();
	  	//	HB_vTask();
    break;
    case MMOT_DIR_RIGHT:
		//Initialize HB module
		//HB_vInit();
        //HB_vHBControl(eHB4,eHB_LON_HOFF);
		//Turn on the low side of half bridge driver 3
	    HB_vHBControl(eHB3,eHB_LON_HOFF);
	 	//Turn on the high side of half bridge driver 2
	  	HB_vHBControl(eHB2,eHB_LOFF_HON);
	    // HB_vHBControl(eHB3,eHB_OFF);
		HB_vUpdateControl();
		//	HB_vTask();
    break;
    case MMOT_DIR_LEFT:
		//Initialize HB module
       	//HB_vInit();
		//Turn on the low side of half bridge driver 2
	    HB_vHBControl(eHB2,eHB_LON_HOFF);
	 	//HB_vHBControl(eHB4,eHB_LOFF_HON);
		//Turn on the high side of half bridge driver 3
	  	HB_vHBControl(eHB3,eHB_LOFF_HON);
	    //HB_vHBControl(eHB3,eHB_OFF);
		HB_vUpdateControl();
		//	HB_vTask();
    break;
#endif

#if defined _RIGHT_MIRROR
	case MMOT_DIR_RIGHT :
		//Initialize HB module
		//HB_vInit();
     	//HB_vHBControl(eHB3,eHB_LON_HOFF);
		//Turn on the low side of half bridge driver 4
	  	HB_vHBControl(eHB4,eHB_LON_HOFF);
	 	//Turn on the high side of half bridge driver 2	
	  	HB_vHBControl(eHB2,eHB_LOFF_HON);
	    //HB_vHBControl(eHB4,eHB_OFF);
	  	HB_vUpdateControl();
	   	//	HB_vTask();
	break;
    case MMOT_DIR_LEFT:
		//Initialize HB module
		//HB_vInit();
		//Turn on the low side of half bridge driver 2
        HB_vHBControl(eHB2,eHB_LON_HOFF);
	 	//HB_vHBControl(eHB3,eHB_LOFF_HON);
		//Turn on the high side of half bridge driver 4
	  	HB_vHBControl(eHB4,eHB_LOFF_HON);
	  	//HB_vHBControl(eHB4,eHB_OFF);
	  	HB_vUpdateControl();
	    //	HB_vTask();
    break;
    case MMOT_DIR_DOWN:
		//Initialize HB module
		//HB_vInit();
        //HB_vHBControl(eHB4,eHB_LON_HOFF);
		//Turn on the low side of half bridge driver 3
	  	HB_vHBControl(eHB3,eHB_LON_HOFF);
	 	//Turn on the high side of half bridge driver 2
	  	HB_vHBControl(eHB2,eHB_LOFF_HON);
	    // HB_vHBControl(eHB3,eHB_OFF);
		HB_vUpdateControl();
		//	HB_vTask();
    break;
    case MMOT_DIR_UP:
       	//Initialize HB module
		//HB_vInit();
		//Turn on the low side of half bridge driver 2
	  	HB_vHBControl(eHB2,eHB_LON_HOFF);
	 	//HB_vHBControl(eHB4,eHB_LOFF_HON);
		//Turn on the high side of half bridge driver 3
	  	HB_vHBControl(eHB3,eHB_LOFF_HON);
	    //HB_vHBControl(eHB3,eHB_OFF);
		HB_vUpdateControl();
		//	HB_vTask();
     break;
#endif

    }
	
}
/**
  * @brief  This function configure half bridges for glass drive and memory brake operation 
  * @param  None
  * @retval None
*/
void ZMDIMmotHBBrake(void){

    // MmotState.sub.error = FALSE;
    // MmotState.sub.state = MMOT_STATE_IDLE ;
	
	/*switch (MmotGetDirection()){
	case MMOT_DIR_LEFT:
	case MMOT_DIR_RIGHT:
    case MMOT_DIR_UP:
    case MMOT_DIR_DOWN:*/  {
    //	HB_vHBControl(eHB2,eHB_OFF);
	HB_vHBControl(eHB3,eHB_OFF);
	HB_vHBControl(eHB4,eHB_OFF);
	//	HB_vHBControl(eHB2,eHB_LON_HOFF);
	HB_vUpdateControl();
	HB_vInit();
	//	HB_vTask();
    // break;
    }
	 
}
/**
  * @brief  This function configure and initializes drivers of half bridges ,high side switches , EC ans PWM 
  * @param  None
  * @retval None
*/
void ZMDIApplInit( void){ 
	//Initialize ECM Driver
  	ECM_vInit();
	//Initialize HS Driver
	HS_vInit();
	//Initialize HB Driver
	HB_vInit();
	//Initialize PWM Driver
	PWM_vInit();

//TODO : This part is kept only for reference. Please remove it before formal release
#ifdef UNKNOWN
  	// After reset mux=0 is selected and no delay is required
  	CurrMoniState = CURR_MONI_STATE_INIT;

 	 // Activate LIN TXD permanent dominant detection
  	gL99CtrlReg3.sig.LINTXDTout = 1;
	
	gL99CtrlReg3.sig.OUT1_OCR = 1; //OVER current recovery bit enabled
	gL99CtrlReg3.sig.OUT5_OCR = 1; //OVER current recovery bit enabled
	gL99CtrlReg3.sig.OCRFREQ = 1;	//Over Current Frequency set to 3khz for 50ms

  	// disable automatic recovery from under/overvoltage without clearing the status register.
 	gL99CtrlReg3.sig.OVUVR = TRUE;
  
 	UBAT_MEAS_SWITCH(1); // for Ubat measurement with optimized quiescent current
  	L99UpdateOutputs();

  	// Check if watchdog reset occured
  	L99UpdateStatus();
  	l_u8_wr_ERR_ST_EXMI_XH_LIN( LIN_ERR_IDX_WATCHDOG, L99_SPI_WDFAIL()? LIN_ERR_ERR:LIN_ERR_OK);

  	TimerDelayUs( UBAT_MEAS_INIT_DELAY_US); // for safety reasons, wait until Ub is stable after switched on
#endif
}
 

/**
  * @brief  This function is used by the application to get values from the currentmonitor.
  * @param  uint8 MuxVal    : requested channel for currentmonitor
  *			uint16* pCurrent: pointer to variable where to store the measured 10 bit adc value.
  * @retval Std_ReturnType : E_OK if no error, otherwise E_NOT_OK
*/

Std_ReturnType ZMDIApplReadCurrentMonitorRaw( uint8 MuxVal, uint16* pCurrent){

   Std_ReturnType ret;

   /*if( (CurrMoniState != CURR_MONI_STATE_IDLE)){
    *pCurrent = 0; // in case of error the current is set defined to zero. So no overcurrent error is detected.
    ret = E_NOT_OK;    
   }	  
   else*/{ // return last measured value
   // *pCurrent = ((AdcRead(MuxVal)*10)/10.5);//*1000*(110/100))*RESCODEHB3_4;//CurrMoniAdcValue;
   *pCurrent = (AdcRead(MuxVal)*1000*(110/100))*RESCODE;
	
   //	 temp1 = MuxVal;

   ret = E_OK;    
  }
  
  return ret; 
}
/**
  * @brief  This function is used by the Powerfold-ISR to measure the rush-in current.
  * @param  uint8 MuxVal    : requested channel for currentmonitor
  * @retval uint16: 10 bit conversion result.
*/
//TODO : This part is kept only for reference and not used. Please remove it before formal release 
uint16 ZMDIApplReadCurrentMonitorRawPfldIsr( uint8 MuxVal){
 #if 0 
  if( MuxVal == gL99CtrlReg4.sig.CM_SELECT){    
    if( CurrMoniState == CURR_MONI_STATE_SWITCHED){
      if( TimerGetElapsedUs( CurrMoniSwitchTimeStart) > L99_CURRENT_MONITOR_SWITCH_DELAY_US){
        CurrMoniState = CURR_MONI_STATE_IDLE;      
      }
    }
    if( CurrMoniState == CURR_MONI_STATE_IDLE){
      return AdcReadCurrentMonitor();
    }
  }
   #endif
  return 0; // in case of error the current is set to zero.
 
}
/**
  * @brief  This function uses the states of the applications which are used for priorization of outputs to be set
  * @param  None
  * @retval None
*/
//TODO: Temporary variable..Remove it in final clean up
uint8_t tempPFstatus;
//TODO : This code has few parts which are kept only for reference . Please remove it before formal release 
void ZMDIApplBeforeWriteOutputs( void){
	//   uint8 next_cm = 0;//gL99CtrlReg4.sig.CM_SELECT; // to detect changes
  	tempPFstatus = PfldIsActive();
 	if( PfldIsActive()){ // select powerfold with highest priority
    /* if( PfldGetDirection() == PFLD_DIR_DRIVE){
      PFLD_MOVE_DRIVE();
      next_cm = PFLD_DRIVE;
    }
    else{ // to park
      PFLD_MOVE_PARK();
      next_cm = PFLD_PARK;
    }*/
	//Now powerfolg state machines are modified which will decide the movement in mmotcyclic itself 
	// and so the above code is not implemented in this part.
	//can be inclueded in future
   }
   else if( MmotIsActive()){ // Select glass motor drive with next priority
   	 ZMDIMmotHBConfig();
	// next_cm = MmotDriveCfg[MmotGetDirection()].MuxChannel;
   }
   else{ // lower priority current measurements
     if( PfldIsBraking()){ // Brake power fold motor
	 //      PfldMotorBrake();
	 }
     else if( MmotIsBraking()){ // Brake mirror adjustment motors
       ZMDIMmotHBBrake();
	 }
     else{
       MOTORS_IDLE(); // All motors idle
     }

   }
#if 0
  uint8 next_cm ;//= gL99CtrlReg4.sig.CM_SELECT; // to detect changes
  
  /*if( PfldIsActive()){ // select powerfold with highest priority
    if( PfldGetDirection() == PFLD_DIR_DRIVE){
      PFLD_MOVE_DRIVE();
      next_cm = PFLD_DRIVE;
    }
    else{ // to park
      PFLD_MOVE_PARK();
      next_cm = PFLD_PARK;
    }
  }	*/
  #if 1
   if( MmotIsActive()){ // Select glass motor drive with next priority
    //gL99CtrlReg1.word = (gL99CtrlReg1.word & ~MOTOR_HALFBRIDGES) | MmotDriveCfg[MmotGetDirection()].HbOnMask;
	ZMDIMmotHBConfig();
    next_cm = MmotDriveCfg[MmotGetDirection()].MuxChannel;
  }
  #endif
  else{ // lower priority current measurements
   /* if( PfldIsBraking()){ // Brake power fold motor
      PfldMotorBrake();
    }*/
	
     if( MmotIsBraking()){ // Brake mirror adjustment motors
	   ZMDIMmotHBBrake();
      //gL99CtrlReg1.word = (gL99CtrlReg1.word & ~MOTOR_HALFBRIDGES) | MmotBrakeHbOnMask[MmotIsDirectionY()];
    }
    else{
      MOTORS_IDLE(); // All motors idle
    }
	if( EcIsHighsideActive()){
     	next_cm = EC_VM;
    }
   #if 0
    if( HtrIsActive()){
      next_cm = HTR_CM;
    }
	 
    else if( EcIsHighsideActive()){
      next_cm = L99_OUT7;
    }
	

    else if( Hc2IsActive()){
      next_cm = HC2_CM;
    }
	#endif
  }
 #if 0 
  // Extended halfbridge diagnostics may change active halbridge outputs, if no request is active
#if HBDIAG_EXTENDED==ON
  HbDiagCyclic();
#endif
  
  if( (next_cm != gL99CtrlReg4.sig.CM_SELECT) || (CurrMoniState == CURR_MONI_STATE_INIT)){
    CurrMoniState = CURR_MONI_STATE_SWITCH_REQUEST;
    gL99CtrlReg4.sig.CM_SELECT = next_cm;
  }
  #endif
  #endif
}

//TODO : Remove the following function if it is not required in future
#if 0
/*=================================================================================================================================
Function name :  L99ApplAfterWriteOutputs
Name          :  L99 application after writing of outputs
Description   :  Must be called after writing control registers of the L99MM70.
                 The timer for the CM-Mux delay must be started after the SPI commands have finished.
Interfaces  
  Parameter(s):  no
  Returnvalue :  no
Version         :
  1.0  APR Creation
=================================================================================================================================*/
void ZAMCApplAfterWriteOutputs( void){
  if( CurrMoniState == CURR_MONI_STATE_SWITCH_REQUEST){
    CurrMoniSwitchTimeStart = TimerGetUs();
    CurrMoniState = CURR_MONI_STATE_SWITCHED;
  }
}





/*=================================================================================================================================
Function name :  L99ApplDbgReadCurrentMonitor
Name          :  L99 application debugging read current monitor
Description   :  Provides the information of the last current measurement for debugging purposes.
                 The 10 bit adc-value and last channel are provided by this function.
Interfaces  
  Parameter(s):  uint16* pCurrent: pointer to variable where to store the measured 10 bit adc value.
  Returnvalue :  uint8 : channel of last current measurement
Version         :
  1.0  APR Creation
=================================================================================================================================*/
#if _DEBUG==ON // for debugging only  
uint8 ZAMCApplDbgReadCurrentMonitor( uint16* pCurrent){
  *pCurrent = CurrMoniAdcValue;
  return (uint8)CurrMoniState;
}
#endif

#endif




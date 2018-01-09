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
 *  @file      Mpot.c
 *
 *  @desc      Sourcefile for mirrorglass drive potentiometers
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
#define _MPOT_C_

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
#include "Mpot.h"
#include "params.h"
#include "debounce.h"
#include "LinAppl.h"

#define POTI_XY_ERR_LIMIT_LOW  (255.*MPOT_ERROR_TH_PERCENT/100.) // difference from 0	 // current val 7
#define POTI_XY_ERR_LIMIT_HIGH (255.*(100-MPOT_ERROR_TH_PERCENT)/100.) // difference from 0	 // = 247
#define POTI_SUP_ERR_LIMIT_MIN (255.*MPOT_ERROR_SUP_MIN/100.)							// = 221
#define POTI_SUP_ERR_LIMIT_MAX (255.*MPOT_ERROR_SUP_MAX/100.)							 //246

#define PER_VAL225_CONV         2.55	 //BR added to match values
#define ADC_MAX_COUNT               1023
static typeDebounceState PotiDiagState;
static typeDebounceCnt PotiDiagCnt;
static const typeDebounceCfg PotiDiagCfg = 
// State       |Timer/Counter| Set delay ticks                          | Reset delay ticks                        | Action                   | Action parameter
{&PotiDiagState, &PotiDiagCnt, MPOT_ERROR_DEBOUNCE_MS/MPOT_CYCLE_TIME_MS, MPOT_ERROR_DEBOUNCE_MS/MPOT_CYCLE_TIME_MS, DEBOUNCE_ACTION_LIN_ERROR, (void*)LIN_ERR_IDX_MPOT};

static uint8 MpotGet( uint8 channel, boolean* pFailed);

static struct{
  unsigned char x :1;
  unsigned char y :1;
}PotiInvert;

volatile uint8 pot_x1, pot_y1;

/**
  * @brief  This function initializes Mirror glass drive potentiometer variables to reset state.
  * @param  None
  * @retval None
*/
void MpotInit( void){
  DebounceInit( &PotiDiagCfg, 1);
  PotiInvert.x = MPOTX_INVERT==1;
  PotiInvert.y = MPOTY_INVERT==1;
}

/**
  * @brief  This function must be called cyclically every MPOT_CYCLE_TIME_MS.
  *         Updates application (checks inputs and updates statemachine and outputs) and diagnostic functions for potentiometers
  * @param  None
  * @retval None
*/


void MpotCyclic( void){
  volatile  uint8 pot_x, pot_y;
  
  boolean failed;
  failed = FALSE;
// mpot SUP CAL REMOVED as it is internal in IntLIN so not required  


  // Calculate new, potivalues with compenated preresistor.
  
  
  pot_x1 = pot_x = MpotGet( ADC_CH_MPOTX ,&failed);
  pot_y1 = pot_y = MpotGet( ADC_CH_MPOTY ,&failed);
  	 
  // Calculation of Potentiometer failure
  if( DebounceUpdate( &PotiDiagCfg,
    FALSE,       // Condition for immediate detection of Poti error (not used)
    failed,      // Condition for delayed detection of Poti error
    FALSE,       // Condition for immediate reset of Poti error (not used)
    failed==0)){ // Condition for delayed reset of Poti error  
#ifdef _POTIVALUES_ON_ERROR_SNV
    // If failure is qualified, set values to SNV
    l_u8_wr_PO_AVL_EXMI_XH_AX_RH_LH_LIN( LIN_POTI_SNV);
    l_u8_wr_PO_AVL_EXMI_XH_AX_UP_DWN_LIN( LIN_POTI_SNV);
  }
  else{
#else // Coordinates on LIN always actual values, errors not signalled with "signal not available"
  }	 
  #endif 
  {

    // Copy to lin. For further usage the value from the LIN buffer is used (GET_MPOT_ACTUAL_X, GET_MPOT_ACTUAL_Y)
    if( PotiInvert.x){
      l_u8_wr_PO_AVL_EXMI_XH_AX_RH_LH_LIN( pot_x);
    }
    else{
	  #if defined _RIGHT_MIRROR
      	l_u8_wr_PO_AVL_EXMI_XH_AX_RH_LH_LIN( ADC_MAX_LSBS8-pot_x);
	  #endif
	  #if defined _LEFT_MIRROR
      	l_u8_wr_PO_AVL_EXMI_XH_AX_RH_LH_LIN(pot_x);
	  #endif

    }

    if( PotiInvert.y){
      l_u8_wr_PO_AVL_EXMI_XH_AX_UP_DWN_LIN( pot_y);
    }
    else{
      l_u8_wr_PO_AVL_EXMI_XH_AX_UP_DWN_LIN( ADC_MAX_LSBS8-pot_y);
    }
  }
}
/**
  * @brief  This function Performs 10 bit AD-conversion for the specified channel and checks for errors. 
  *	        The ADC result is adjusted to fit in an 8-bit value. Voltage on MPOT_SUP is used for adjustment.
  * @param  uint8 channel   : AD channel to be used for conversion
  *         uint8 pot_sup   : 8-bit result of AD conversion for MPOT_SUP
  *         boolean* pFailed: pointer to variable for failure detection. Set on error detection - otherwise unchanged.
  * @retval uint8: new, adjusted 8-bit result for potentiometer.
*/
static uint8 MpotGet( uint8 channel,  boolean* pFailed){
  
     volatile uint16 adc , ratio;
     uint8  result;
     float temp;	 
     adc = ADC_tGetADCValueByNdx(channel);//AdcRead( channel);
  	  // ration of mpot_sup/mpotx-y
	 ratio = ((float)adc/ ADC_MAX_COUNT)*100;
 	 // conversion of mpot voltage values to values(0-255) 
	 result = ratio * PER_VAL225_CONV;
	if( result < POTI_XY_ERR_LIMIT_LOW){
      *pFailed = TRUE;
    }
    else if( result > POTI_XY_ERR_LIMIT_HIGH){
      *pFailed = TRUE;								 
    }  
  	 
  return result;
}



/*-- End of File --*/

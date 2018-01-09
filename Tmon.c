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
 *  @file      Tmon.c
 *
 *  @desc      The following file is to implement temperature monitoring.
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

/**********************************************************************************************************************************
Project     :  L7-Mirror
Filename    :  Tmon.c
Brief descr.:  Sourcefile for temperature monitoring
Copyright   :  
Remarks     :  
Version
  1.0.0  18.09.2009  UF  Creation
**********************************************************************************************************************************/
#define _TMON_C_

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
#include "error.h"
#include "Tmon.h"
#include "Hardware.h"
#include "Params.h"
#include "LinAppl.h"


#define T_high_limit 36 //Raw value (TmonTempRaw) obtained at 125°C @R...Shutoff happened at 124.8°C
#define T_low_limit 46 // Raw value (TmonTempRaw) obtained at 115°C @R
#define ADC_CH_EXT_TEMP      eNdxAIN_INTTMP	   // FOR ADC internal temperature
#define CHECK_TEMPERATURE_ABOVE( Traw, T_high_limit) (Traw < ( T_high_limit))
#define CHECK_TEMPERATURE_BELOW( Traw, T_low_limit) (Traw > ( T_low_limit))
//#define CHECK_TEMPERATURE_ABOVE( Traw, T_high_limit) FALSE
//#define CHECK_TEMPERATURE_BELOW( Traw, T_low_limit) TRUE

extern uint8_t ADC_CHANNEL_MUX ;

uint8_t TMON_High(void);
/**
  * @brief  Initializes the variables used for temperature monitoring
  * @param  None
  * @retval None
*/
void TmonInit( void){
  TmonState = TMON_OK;
}
/**
  * @brief  Reads MCU internal temperature and updates the states for overtemperature detection.The ZAMC temperature warning flag is also considered.
  * @param  None
  * @retval None
*/
void TmonCyclic( void){

  //ADC_CHANNEL_MUX = eINTTMP_MUX_NDX;
  //ADC reads external temperature sensor value
  TmonTempRaw = AdcRead( ADC_CH_EXT_TEMP);
  //Checking whether temperature is above the operating temperature 			
  if( TMON_High() || CHECK_TEMPERATURE_ABOVE( TmonTempRaw, T_high_limit)){
    TmonState = TMON_ERROR;
  }
  //Checking whether temperature decreases to allowed operating limit inorder to restart the ECU 
  else if( CHECK_TEMPERATURE_BELOW( TmonTempRaw, T_low_limit)){    
    TmonState = TMON_OK;
  }
  l_u8_wr_ERR_ST_EXMI_XH_LIN( LIN_ERR_IDX_OVERTEMP, TmonState == TMON_ERROR? LIN_ERR_ERR:LIN_ERR_OK);
}
/**
  * @brief  Reads MCU internal temperature Status flag and updates the states for overtemperature 
  * @param  None
  * @retval 0 if Overtemperature flag is not set
  *			1 if overtemperature flag is set
*/
uint8_t TMON_High(void)
{
	volatile uint8_t* TMON_stat;
	//Standard SPIread process of RSTSTAT register
	SPIMgr_eRegRead(RSTSTAT);
	TMON_stat = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(RSTSTAT));
	if((*TMON_stat & 0x08) == 0x08) {
		return 1;		
	 }
	else{
		return 0;
	 }
}


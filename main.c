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
 *  @file      main.c
 *
 *  @desc      The following file is to implement Integrated LIN ECU control logic.
 *
 *  @document  Integrated_LINBus_Software_Design.docx
 *
 * =============================================================================
 *  Revision History
 *  ===============
 *   Date    Initials   Version   	Change
 * 05-11-16	   APR		 0.0.1    	Initial version
 * 10-10-17    SH&BG     0.0.2    	
 * 12-10-17    SH&BG     0.0.3       
 * 12-10-17    SH&BG     0.0.4      
 * =============================================================================
 */

 /* Include-Files ################################################################################################################*/
#include "stdint.h"
#include "std_init.h"
#include "ZMDCM0.h"
#include "SPI.h"
#include "ADC.h"
#include "SBC.h"
#include "SPIMgr.h"
#include "TIMER.h"
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
#include "pfld.h" 
#include "WDG.h"
#include "Mmot.h"
#include "timer.h"
#include "htr.h"
#include "ec.h"
#include "HS.h"
#include "Mpot.h"
#include "htr.h"
#include "stdlib.h"
#include "string.h"
#include "version.h"
#include "Hc2.h"
#include "LinAppl.h"
#include "Interrupts.h"

#pragma push
	#pragma anon_unions
	#define __IO volatile
	#include "AHB_LIN.h"
	#define AHB_LIN_IRQn (SW_LIN_IRQn)
	#include "STick.h"
	#include "GPIO.h"
#pragma pop

/* Defines ################################################################################################################*/
 #define Read 	0x20
 #define Write 	0
 #define DELAY 	1
// This is required to get same codesize for left and rigt mirror.
 const uint8 LIN_CTRL_SELECT_ASP_XXXX = 

 #ifdef _LEFT_MIRROR // left mirror
	LIN_CTRL_SELECT_ASP_LEFT;
 #endif

 #ifdef _RIGHT_MIRROR // right mirror
	LIN_CTRL_SELECT_ASP_RIGHT;
 #endif

/*################################################################################################################*/

extern uint8_t pfld_active_flag;

uint8_t bVTFailure;
uint8_t	  MainClock;			
		
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
   volatile eHS_ShortCircuitType eLoadStatus;
 volatile uint8_t u8ErrorCnt;
 static volatile teECM_DiagStatus ECM_eLoadStatus;
int main(void)
	{
		// Initialize basic driver modules
		//Initialize ISR counter
		InterruptsInit();
		//Initialize GPIO module
		GPIO_vInit();
		//Initialize SPI module	 
		SPI_vInit();
		//Initialize System timer module
		SysTick_vInit();
		//Initialize SPI Manager module
		SPIMgr_vInit();
		//Initialize ADC module
 		ADC_vInit();
		//Initialize 32 bit timer module
		TMR32_vInit();
		//Allows SPI manager to start
		SPIMgr_vStart();
		//Initialize LIN transever module
		LINCom_vInit();
		//Initializes Half bridges,High side switches,Electrochrome module and PWM module
		ZMDIApplInit();
		//Initialize Watch dog timer module
		WDG_vInit();
    	PWM_vInit(); //TODO: It is already added in ZMDIApplInit. Please remove it in future

		// Initialize functional modules
		//Initialize Halfbridges for powerfold functionality
		PfldInit();
		//Initialize High side switch for heater functionality
	    HtrInit();
		//Initialize Voltage monitering functionality
	    VmonInit();
		//Initialize High side switch for HC2 functionality
	    Hc2Init();
		//Initialize Halfbridges for glassdrive and memory functionality
	   	MmotInit();

		 ECM_vInit();
		//TODO: Add MpotInit and TmonInit and verify functionality

		//Calculate the main clock cycles for timer iterations
     	MainClock = MAIN_CYCLE_MS/TIMER_CYLE_MS;
			eLoadStatus = eHS_Normal;
			u8ErrorCnt=0;
		
	while(1)
	{
	    //Timer event set for 10ms loop cycle  
		if (TMR32_u32IsFlagSet() == TMR_EVENT_SET)
		{  
			//Stops the LIN Communication temporarily for making SPI BUS Free 
			//TODO: This is not desired way of implementation.Modify algorithm in right way
		//	LINCom_vStop();
			//Process ready messages If the status of SPI manager is active		
			SPIMgr_vTask();	
			//Dummy function call
				  
			//TODO: Can be removed in future if not required     
		//	ADC_vTask();
			//Reads ubat related voltages and updates the states for over- and undervoltage detection
	    //	VmonCyclic();
			//Temperature monitoring cyclic call.Reads MCU internal temperature and updates the states for overtemperature detection
		//	TmonCyclic();
			//Cyclic Calculation of Potentiometer values
	    //	MpotCyclic();
			//Updating the failure/succes condition of Voltage Input and operating temperature sensor value
		//	bVTFailure  = VmonGetFailure(); //|| TmonGetFailure();		   //temperature not activated 
			//Function to check for LIN new messages and also timeout of messages
		//	LinApplUpdateBrcCtrlAsp(); 
			//Cyclic function for glassdrive and memory requests and its excecution 
		//	MmotCyclic();
			//Cyclic function for powerfold requests and its excecution
		//	PfldCyclic(); 
			//Cyclic function for EC requests and its excecution
	    //	EcCyclic();
			
			//Cyclic function for HC2 requests and its excecution
		// 	Hc2Cyclic();
			//Cyclic function for Heater requests and its excecution
		//    HtrCyclic();
			//Checks if there are new drivers to enables/disables and writes the new values for duty cycle and frequency if any. Should be called cyclic as per ZAMC driver SW requirement
	 	//	PWM_vTask();
			//Checks if there are new drivers to enables/disables and writes the new values for duty cycle and frequency if any. Should be called cyclic as per ZAMC driver SW requirement
	 //		HS_vTask();
			//Checks if there are new drivers to enables/disables and writes the new values for duty cycle and frequency if any. Should be called cyclic as per ZAMC driver SW requirement
	 	//	HB_vTask(); 
			//Checks if there are new drivers to enables/disables and writes the new values for duty cycle and frequency if any. Should be called cyclic as per ZAMC driver SW requirement
	 		ECM_vTask();
			ECM_eLoadStatus	  = ECM_eGetDiagStatus();
			//Checks if there are new drivers to enables/disables and writes the new values for duty cycle and frequency if any. Should be called cyclic as per ZAMC driver SW requirement
	 	  	WDG_vTask();
			//The states of the applications are used for priorization of outputs
	//		ZMDIApplBeforeWriteOutputs();
			//Checks and Update Node-error status
			//TODO:Test for Sleep-Mode has to be implemented 
	//		LinApplUpdateTxBuffers();
			//Reintializing LIN module after initial start
	//		LINCom_vInit();
			//Restarting LIN Transceiver module
	//		LINCom_vStart(); 
	//	 	
		
			
   }
   //System clock is used for Powerfold to measure Inrush current for every 1msec
   	if (SysTick_u32IsFlagSet() == SYSTICK_FLAG_SET)
	{
	   //Funtion to enable Powerfold Current measurement in every msec
	   PfldTimerIsr();
	} 
   }
}

// This callback is copied from SPIMgr.c
//TODO: This can be placed in dummy_callback.c
void IRQSTAT_Callback(eSPIMgr_OperationType eOp,eSPIMgr_OperationStatus eStatus)
{

}

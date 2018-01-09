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
 *  @file      Interrupts.c
 *
 *  @desc      Sourcefile for interrupt functionality 
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
#define _INTERRUPTS_C_

#include "stdint.h"
#include "std_init.h"

#define disableInterrupts()   __disable_irq()
#define enableInterrupts()	   __enable_irq()

static unsigned char counterISRdisable;
void InterruptsInit(void);
void InterruptsSuspend(void);

/*=================================================================================================================================
Function name :  
Name          :  
Description   :  
Interfaces  
  Parameter(s):  no
  Returnvalue :  no
Version         :
  1.0  04.11.2013 UF Creation
=================================================================================================================================*/
/**
  * @brief  This function Initializes interrupt counter variable 
  * @param  None
  * @retval None
*/
void InterruptsInit(void){
  counterISRdisable = 1;
}
/**
  * @brief  This function disables interrupt
  * @param  None
  * @retval None
*/
void InterruptsSuspend(void){
 if (counterISRdisable == 0){
   disableInterrupts();
 }
 counterISRdisable++;
}
/**
  * @brief  This function restores interrupt
  * @param  None
  * @retval None
*/
void InterruptsRestore(void){
 if (counterISRdisable){
   counterISRdisable--;
 }
 if (counterISRdisable == 0){
   enableInterrupts();
 }
}

/*-- End of File --*/

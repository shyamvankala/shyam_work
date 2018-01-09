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
 *  @file      LinAppl.c
 *
 *  @desc      Sourcefile for Sourcefile for Errorhandling (arrays for suberrors)
 *             ourcefile for LIN-driver addon.Initialization and special handling 
 *  		   of LIN frames
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
#define _LINAPPL_C_

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
#include "error.h"
#include "Tmon.h"
#include "params.h"
#include "Applinit.h"
#include "debounce.h"
#include "LinAppl.h"
#include "vmon.h"
#include "pfld.h"
#include "htr.h"
#include "ec.h"
#include "mmot.h"
#include "interrupts.h"

// for calculation of NOD_ERR_EXMI_XH_LIN
static boolean NodeErrChanged;
static typeTimerValue BRC_CTR_ASP_LIN_lastRxTime;
static boolean BRC_CTR_ASP_LIN_timeout;
// For detection of bootloader request
static boolean BRC_CTR_ASP_LIN_detected;
// for calculation of NOD_ERR_EXMI_XH_LIN
//#if _DEBUG==ON // for debugging only  
# define HOST_2_LIN_16(val) (((val)>>8)|((val)<<8))
# define HOST_2_LIN_32(val) (((val)<<24)|(((val)&(0xFF00))<<8)|(((val)&(0xFF0000))>>8)|((val)>>24))
//#endif

//#if _DEBUG==ON // for debugging only  
static union{
  uint8 bytes[8];
  struct{
    uint8 SMDCTRLReg;
	uint8 IRQCTRLReg;
	uint8 HBDCTRLReg;
	uint8 HSDCTRLReg;
	uint8 ECMCTRLReg;
	uint8 ADCCTRLReg;
	uint8 LINCTRLReg;
	uint8 T32CTRLReg;
   
  }regs;
}dbgZAMCctrl;
static union{
  uint8 bytes[8];
  struct{
    uint8 RSTSTATReg;
	uint8 IRQSTATReg;
	uint8 HBDSTATReg;
	uint8 HSDSTATReg;
	uint8 ADCSTATReg;
	uint8 LINSTATReg;
	uint8 temp1;
	uint8 temp2;
   
  }regs;
}dbgZAMCSTAT;
static enum{
	SMD =0,
	IRQ,
	HBD,
	HSD,
	ECM,
	ADC,
	LIN,
	T32,
	RST 
};//CtrlRegOdr;

static union{
  uint8 bytes[8];
  uint16 words[4];
  struct{
    uint16 tmon_adc :10;
    uint16 tmon_state :6;
    uint16 vmon_adc :10;
    uint16 vmon_uv :2;
    uint16 vmon_uv_htr :2;
    uint16 vmon_ov :2;
    uint16 cm_adc :10;
    uint16 cm_mux : 4;
    uint16 cm_state :2;
    uint16 umot_adc :10;
    uint16 pfld_err0 :2;
    uint16 pfld_err1 :2;
    uint16 pfld_err2 :2;
  }sig;
}dbgStat1;

static union{
  uint8 bytes[8];
  uint16 words[4];
  struct{
    uint16 ec_diag_out :1;
    uint16 ec_diag_in :1;
    uint16 res1 :8;
    uint16 htr_state :2;
    uint16 htr_oc :2;
    uint16 htr_opnld :2;
    uint16 hc2_adc :10;
    uint16 hc2_state :6;
    uint8 ec_state :8; // only 2 Bits used
    uint8 mm_state :8;
    uint8 mm_err0 :2;
    uint8 mm_err1 :2;
    uint8 mm_err2 :2;
    uint8 mm_err3 :2;
    uint8 mm_err4 :2;
    uint8 mm_err5 :2;
    uint8 pfld_state :4;
  }sig;
}dbgStat2;

static union{
  uint8 bytes[8];
  uint16 words[4];
  struct{
    uint8 sw_version_sub;
    uint8 hc2_err0 : 2;
    uint8 hc2_err1 : 2;
    uint8 hc2_err2 : 2;
    uint8 hc2_err3 : 2;
    uint8 hc2_err4 : 2;
    uint8 hc2_err5 : 2;
    uint8 hc2_err6 : 2;
    uint8 ec_err0 : 2;
    uint8 ec_err1 : 2;
    uint8 ec_err2 : 2;
    uint8 ec_err3 : 2;
    uint8 res : 2;
    uint8 pot_sup_adc;
  }sig;
}dbgStat3;

static union{
  uint8 bytes[8];
  uint16 words[4];
  struct{
    uint16 PwmPeriodLsbs;
    uint16 PwmOnTimeLsbs;
    uint16 PfldPeakCurr;
    uint16 PfldBlockCurr;
  }sig;
}dbgStat4;

static union{
  uint8 bytes[8];
  uint16 words[4];
  struct{
    uint32 brightness;
    uint16 bright_req;
    uint16 bright_prev;
  }sig;
}dbgStat5;

static union{
  uint8 bytes[8];
  uint16 words[4];
  struct{
    uint32 htr_ui;
    uint8 HbDiagErrors;
    uint8 mm_err6   :2;
    uint8 pfld_err3 :2;
  }sig;
}dbgStat6;

uint8_t STAT_Reg_Read(uint8_t value);
//#endif

/**
  * @brief  LIN update receive LIN frame BRC_MEM_POS_EXMI_LIN
  *         Checks, if the LIN frame BRC_MEM_POS_EXMI_LIN was updated. If this is the case the function 
  *         returns E_OK. If the frame was not updated, the function returns E_NOT_OK.
  * @param  None
  * @retval Std_ReturnType: E_OK     -> LIN frame was received
  *                         E_NOT_OK -> LIN frame was not received
*/
Std_ReturnType LinApplUpdateBrcMemPos( void){
  Std_ReturnType ret;
  if( l_flg_tst_RxBRC_MEM_POS_EXMI_LIN()){
    l_flg_clr_RxBRC_MEM_POS_EXMI_LIN();
    ret = E_OK;
  }
  else{
    ret = E_NOT_OK;
  }
  return ret;
}

/**
  * @brief  LIN update receive LIN frame BRC_CTR_ASP_LIN
  *         The functions checks for receiption of the LIN frame BRC_CTR_ASP_LIN. If it was not updated a timer is used to
  *         set default values after timeout has expired.
  * @param  None
  * @retval None
*/
void LinApplUpdateBrcCtrlAsp( void){
  // Must be done with interrupts disabled to avoid inconsistent data.
  // Otherwise in case near timeout the data written by LIN-ISR might be overwritten with 0xFF.
  // When LIN-ISR is triggered after l_flg_tst_RxBRC_CTR_ASP_LIN()==0 and then timeout is detected.
  InterruptsSuspend();
  if( l_flg_tst_RxBRC_CTR_ASP_LIN()){
    l_flg_clr_RxBRC_CTR_ASP_LIN();
    BRC_CTR_ASP_LIN_timeout = FALSE;	   
    BRC_CTR_ASP_LIN_lastRxTime = TimerGet();
    BRC_CTR_ASP_LIN_detected = TRUE;
  }
  else{ // No new messagedata available
    if( BRC_CTR_ASP_LIN_timeout){ // Already timeout
    }
    else if( TimerGetDiffTicks( BRC_CTR_ASP_LIN_lastRxTime) > t_in_ms(TIMEOUT_BRC_CTR_ASP_LIN_MS)){
      memset( &l_pFrameBuf[LIN_BYTE_OFFSET_BRC_CTR_EXMI_LIN], 0xFF, 8);  
      BRC_CTR_ASP_LIN_timeout = TRUE;
    }    
  }
  InterruptsRestore();
}
/**
  * @brief  IN unsigned 8 bit write signal ERR_ST_EXMI_XH_LIN
  *         Checks if a specific error in transmit LIN frame ERR_ST_EXMI_XH_LIN has changed. If this is the case, the new
  *         error is copied to the transmit buffer. Also the signal l_u8_wr_NOD_ERR_EXMI_XH_LIN is set to "error state changed".
  * @param  uint8 err_idx: 
  *         uint8 new_val:
  * @retval None
*/
void l_u8_wr_ERR_ST_EXMI_XH_LIN(uint8 err_idx, uint8 new_val){
  if( new_val != l_u8_rd_ERR_ST_EXMI_XH_LIN( err_idx)){
    l_pFrameBuf[LIN_BYTE_OFFSET_ERR_ST_EXMI_XH_00_LIN+err_idx/4] = (l_pFrameBuf[LIN_BYTE_OFFSET_ERR_ST_EXMI_XH_00_LIN+err_idx/4] & ~(3 << ((err_idx & 0x03)*2))) | (new_val << ((err_idx & 0x03)*2));
    InterruptsSuspend();
    l_u8_wr_NOD_ERR_EXMI_XH_LIN( LIN_NOD_ERR_CHANGE);
    l_flg_clr_TxNOD_ERR_EXMI_XH_LIN();
    InterruptsRestore();
    NodeErrChanged = TRUE;
  }
}
/**
  * @brief  Puts the local buffers for LIN signals to the freescale LIN driver.
  *         Checks for sleep-mode and receiption of EOL_TST_EXMI_RESP_LIN frame.
  *         If required, a jump to the bootloader is performed.
  * @param  None
  * @retval None
*/
//TODO: Clear unnecessary code part after full implementation
void LinApplUpdateTxBuffers( void){
  uint8_least i;

  // check if new status "changed" was requested
  if( NodeErrChanged){
    if( l_flg_tst_TxNOD_ERR_EXMI_XH_LIN()){
      NodeErrChanged = FALSE;
    }
  }
  
  if( NodeErrChanged == FALSE){
    uint8 NodError = LIN_NOD_ERR_NO;
    // Update Node-error status
    for( i = 0; i < 16; i++){
      if( l_u8_rd_ERR_ST_EXMI_XH_LIN( i) == LIN_ERR_ERR){
        NodError = LIN_NOD_ERR_ERR;
        break;
      }
    }
    l_u8_wr_NOD_ERR_EXMI_XH_LIN( NodError);
  }

#if 0
  // Test for Sleep-Mode
  if( !PfldIsActive() && !PfldIsBraking() &&           // Powerfold not in process
    !MmotIsActive() && !MmotIsBraking()){             // Glassmotors not in process
    if( (l_ifc_ioctl( LIN_IFC_SCI1, LIN_IOCTL_DRIVER_STATE, 0) & 0x00FF) == LIN_STATE_BUSSLEEP){ // LIN down
#if _FBLSUP==1 // End of line programming
     // Test for bootloader request
      if( (BRC_CTR_ASP_LIN_detected == FALSE)  // BRC_CTR_ASP_LIN never received
      && (VmonGetUbatRaw() > UBAT_MEAS_VOLTS_TO_LSBS(15))){ // and more than 15V
        ENTER_BOOTLOADER_REQUEST_SET;
        //reset stack pointer (lower byte - because compiler decreases SP with some bytes)
        #ifdef _COSMIC_ 
         _asm("LDW X,  SP ");
         _asm("LD  A,  $FF");
         _asm("LD  XL, A  ");
         _asm("LDW SP, X  ");
         _asm("JPF $8000"); 
        #elif defined  _IAR_
         asm("LDW X,  SP ");
         asm("LD  A,  $FF");
         asm("LD  XL, A  ");
         asm("LDW SP, X  ");
         asm("JPF $8000");  
        #else /*_RAISONANCE_*/
        #pragma ASM
         LDW X,SP
         LD A,0FFH
         LD XL,A
         LDW SP,X
         JPF 08000H  
        #pragma ENDASM
        #endif /* _COSMIC_*/
      }
      else
#endif // _FBLSUP==1
      {
        L99_SPI_ENABLE_STANDBY( 1); // Goto Sleep-Mode (Power is expected to go off)
        L99UpdateOutputs();
        while( (l_ifc_ioctl( LIN_IFC_SCI1, LIN_IOCTL_DRIVER_STATE, 0) & 0x00FF) == LIN_STATE_BUSSLEEP){ // Wait for power off until wakeup is detected - on wakeup continue application
        }
        L99_SPI_ENABLE_STANDBY( 0);
      }
	  
    }
  }
  #endif
}

/**
  * @brief  This function reads the ZAMC control register values 
  * @param  uint8_t value 
  * @retval uint8_t control register read value
*/
//TODO: Remove this function after full implementation
uint8_t CTRL_Reg_Read(uint8_t value) {
	 
	uint8_t* CTRL_Val;
	uint8_t result;
	switch (value){
	case SMD:
			   	SPIMgr_eRegRead(SMDCTRL);
				CTRL_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(SMDCTRL));
				break;
	case IRQ:
				SPIMgr_eRegRead(IRQCTR);
				CTRL_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(IRQCTR));
				break;
	case HBD:
				SPIMgr_eRegRead(HBDCTRL);
				CTRL_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(HBDCTRL));
				break;

	case HSD:
				SPIMgr_eRegRead(HSDCTRL);
				CTRL_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(HSDCTRL));
				break;

	case ECM:
					SPIMgr_eRegRead(ECMCTRL);
				CTRL_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(ECMCTRL));
				break;

	case ADC:
					SPIMgr_eRegRead(ADCCTRL);
				CTRL_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(ADCCTRL));
				break;

	case LIN:
				SPIMgr_eRegRead(LINCTRL);
				CTRL_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(LINCTRL));
				break;

	case T32:
				//SPIMgr_eRegRead(T32CTRL);
				//CTRL_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(T32CTRL));
				break;

	 default:
				break;
	 }
	return result = *CTRL_Val;


} 
/**
  * @brief  This function reads the ZAMC status register values 
  * @param  uint8_t value 
  * @retval uint8_t status register read value
*/
//TODO: Remove this function after full implementation
uint8_t STAT_Reg_Read(uint8_t value) {
	 
	uint8_t* STAT_Val;
	uint8_t result;
	switch (value){
	case RST:
			   	SPIMgr_eRegRead(RSTSTAT);
				STAT_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(RSTSTAT));
				break;
	case IRQ:
				SPIMgr_eRegRead(IRQSTAT);
				STAT_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(IRQSTAT));
				break;
	case HBD:
				SPIMgr_eRegRead(HBDSTAT);
				STAT_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(HBDSTAT));
				break;

	case HSD:
				SPIMgr_eRegRead(HSDSTAT);
				STAT_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(HSDSTAT));
				break;

	case ECM:
				//	SPIMgr_eRegRead(ECMSTAT);
				//STAT_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(ECMSTAT));
				break;

	case ADC:
				SPIMgr_eRegRead(ADCSTAT);
				STAT_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(ADCSTAT));
				break;

	case LIN:
				SPIMgr_eRegRead(LINSTAT);
				STAT_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(LINSTAT));
				break;

	case T32:
				//SPIMgr_eRegRead(T32STAT);
				//STAT_Val = SPIMgr_pu8GetMsgBufferFromAddress(ADDR_TO_INDEX_E(T32STAT));
				break;

	 default:
				break;
	 }
	return result = *STAT_Val;


} 

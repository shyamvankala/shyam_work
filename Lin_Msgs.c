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
 *  @file      Lin_Msgs.c
 *
 *  @desc      Sourcefile for copying LIN Messages from LIN BUS 
 *
 *  @document  Integrated_LINBus_Software_Design.docx
 *
 * =============================================================================
 *  Revision History
 *  ===============
 *   Date    Initials   Version   	Change
 * 07-03-17	   BG		 0.0.1    	Initial version
 
 * =============================================================================
*/                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
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
#include "interrupts.h"
#pragma push
#pragma anon_unions
#define __IO volatile 								  
#include "AHB_LIN.h"
#define AHB_LIN_IRQn (SW_LIN_IRQn)
#include "STick.h"
#include "GPIO.h"
#pragma pop



struct{
		uint8_t	brc_ctr_exmi_lin_sig : 2;
		uint8_t	brc_ctr_exmi_ad_lin_sig : 4;
		uint8_t	brc_ctr_exmi_fmir_lin_sig : 2;
		uint8_t	brc_ctr_ecr_exmi_dip_lin_sig;
		uint8_t	brc_ctr_ht_exmi : 3;
		uint8_t	ctr_dim_dse_lnch_lin;
		uint8_t	ctr_dse_1_lh_warm_lnch_lin_sig : 4;
		uint8_t	ctr_dse_1_rh_warm_lnch_lin_sig : 4;
		uint8_t	ctr_scal_idc_dse_warm_lch_lin_sig;

		}BRC_CTR_EXMI_LIN_FRAME;

//Buffer to store LIN DATA
uint8_t    l_pFrameBuf[LIN_FRAME_BUF_SIZE] =
{
   0xff, /* 0 : 11111111*//* start of frame BRC_MEM_POS_EXMI_LIN */ 
   0xff, /* 1 : 11111111*/ 
   0xff, /* 2 : 11111111*/ 
   0xff, /* 3 : 11111111*/ 
   0xf0, /* 4 : 11111110*//* start of frame BRC_CTR_ASP_LIN */ 
   0xff, /* 5 : 11111111*/ 
   0xff, /* 6 : 11111111*/ 
   0xff, /* 7 : 11111111*/ 
   0xff, /* 8 : 11111111*/ 
   0xff, /* 9 : 11111111*/ 
   0xff, /* 10 : 11111111*/ 
   0xff, /* 11 : 11111111*/ 
   0xff, /* 12 : 11111111*//* start of frame ERR_ST_ASP_LH_LIN */ 
   0xff, /* 13 : 11111111*/ 
   0xff, /* 14 : 11111111*/ 
   0xff, /* 15 : 11111111*/ 
   0xff, /* 16 : 11111111*/ 
   0xff, /* 17 : 11111111*/ 
   0xff, /* 18 : 11111111*/ 
   0xff, /* 19 : 11111111*/ 
   0xff, /* 20 : 11111111*//* start of frame ST_HC2_ASP_LH_LIN */ 
   0x7f, /* 21 : 01111111*/ 
   0xff, /* 22 : 11111111*//* start of frame ST_POS_EXMI_LH_LIN */ 
   0xff, /* 23 : 11111111*/ 
   0xff, /* 24 : 11111111*//* start of frame EOL_TST_EXMI_REQ_LIN */ 
   0xff, /* 25 : 11111111*/ 
   0xff, /* 26 : 11111111*/ 
   0xff, /* 27 : 11111111*/ 
   0xff, /* 28 : 11111111*/ 
   0xff, /* 29 : 11111111*/ 
   0xff, /* 30 : 11111111*/ 
   0xff, /* 31 : 11111111*/ 
   0x00, /* 32 : 00000000*//* start of frame DBG_ASP_LH_L99_CTRL */ 
   0x00, /* 33 : 00000000*/ 
   0x00, /* 34 : 00000000*/ 
   0x80, /* 35 : 10000000*/ 
   0x00, /* 36 : 00000000*/ 
   0x80, /* 37 : 10000000*/ 
   0xa0, /* 38 : 10100000*/ 
   0x00, /* 39 : 00000000*/ 
   0x00, /* 40 : 00000000*//* start of frame DBG_ASP_LH_L99_STAT */ 
   0x00, /* 41 : 00000000*/ 
   0x00, /* 42 : 00000000*/ 
   0x00, /* 43 : 00000000*/ 
   0x08, /* 44 : 00001000*/ 
   0x00, /* 45 : 00000000*/ 
   0x80, /* 46 : 10000000*/ 
   0x00, /* 47 : 00000000*/ 
   0x00, /* 48 : 00000000*//* start of frame DBG_ASP_LH_STAT1 */ 
   0xf8, /* 49 : 11111000*/ 
   0x00, /* 50 : 00000000*/ 
   0x00, /* 51 : 00000000*/ 
   0x00, /* 52 : 00000000*/ 
   0x00, /* 53 : 00000000*/ 
   0x00, /* 54 : 00000000*/ 
   0x00, /* 55 : 00000000*/ 
   0xfc, /* 56 : 11111100*//* start of frame DBG_ASP_LH_STAT2 */ 
   0x03, /* 57 : 00000011*/ 
   0x00, /* 58 : 00000000*/ 
   0x80, /* 59 : 10000000*/ 
   0xfc, /* 60 : 11111100*/ 
   0x00, /* 61 : 00000000*/ 
   0x00, /* 62 : 00000000*/ 
   0x00, /* 63 : 00000000*/ 
   0x00, /* 64 : 00000000*//* start of frame DBG_ASP_LH_STAT3 */ 
   0x00, /* 65 : 00000000*/ 
   0x00, /* 66 : 00000000*/ 
   0xc0, /* 67 : 11000000*/ 
   0x00, /* 68 : 00000000*/ 
   0xff, /* 69 : 11111111*/ 
   0xff, /* 70 : 11111111*/ 
   0xff, /* 71 : 11111111*/ 
   0x00, /* 72 : 00000000*//* start of frame DBG_ASP_LH_STAT4 */ 
   0x00, /* 73 : 00000000*/ 
   0x00, /* 74 : 00000000*/ 
   0x00, /* 75 : 00000000*/ 
   0x00, /* 76 : 00000000*/ 
   0x00, /* 77 : 00000000*/ 
   0x00, /* 78 : 00000000*/ 
   0x00, /* 79 : 00000000*/ 
   0x00, /* 80 : 00000000*//* start of frame DBG_ASP_LH_STAT5 */ 
   0x00, /* 81 : 00000000*/ 
   0xff, /* 82 : 11111111*/ 
   0xff, /* 83 : 11111111*/ 
   0x00, /* 84 : 00000000*/ 
   0x00, /* 85 : 00000000*/ 
   0x00, /* 86 : 00000000*/ 
   0x00, /* 87 : 00000000*/ 
   0x00, /* 88 : 00000000*//* start of frame DBG_ASP_LH_STAT6 */ 
   0xfc, /* 89 : 11111100*/ 
   0xff, /* 90 : 11111111*/ 
   0xff, /* 91 : 11111111*/ 
   0xff, /* 92 : 11111111*/ 
   0xff, /* 93 : 11111111*/ 
   0xff, /* 94 : 11111111*/ 
   0xff, /* 95 : 11111111*/ 
   0x00, /* 96 : 00000000*//* start of frame MasterReq */ 
   0x00, /* 97 : 00000000*/ 
   0x00, /* 98 : 00000000*/ 
   0x00, /* 99 : 00000000*/ 
   0x00, /* 100 : 00000000*/ 
   0x00, /* 101 : 00000000*/ 
   0x00, /* 102 : 00000000*/ 
   0x00, /* 103 : 00000000*/ 
   0x00, /* 104 : 00000000*//* start of frame SlaveResp */ 
   0x00, /* 105 : 00000000*/ 
   0x00, /* 106 : 00000000*/ 
   0x00, /* 107 : 00000000*/ 
   0x00, /* 108 : 00000000*/ 
   0x00, /* 109 : 00000000*/ 
   0x00, /* 110 : 00000000*/ 
   0x00 /* 111 : 00000000*/ 
};

const uint16_t l_pMessageId[LIN_FRAME_COUNT] = 
{
  0x000aU,
  0x000bU,
  0x000cU,
  0x000dU,
  0x000eU,
  0x000fU,
  0xffffU,
  0xffffU,
  0xffffU,
  0xffffU,
  0xffffU,
  0xffffU,
  0xffffU,
  0xffffU,
  0xffffU,
  0xffffU
};


uint8_t l_pFrameId[LIN_FRAME_COUNT] = 
{
  173,
  111,
  207,
  240,
  17,
  251,
  50,
  115,
  180,
  245,
  118,
  55,
  120,
  57,
  60,
  125
};

uint8_t  l_pTxFlagData[LIN_TRANSMIT_FLAG_DATA_SIZE];
uint8_t  l_pRxFlagData[LIN_TRANSMIT_FLAG_DATA_SIZE];
uint8_t    l_pChangedFlagData[LIN_CHANGED_FLAG_DATA_SIZE];
uint8_t  LinReceivedData[8];

/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  BRC_CTR_ASP_LIN_Notification(void)	 // Message direction master to slave
{
	uint8_t u8Len,*pBuf;
	//Getting the length of message
	u8Len =  LIN_FRAME_GET_BUF_LEN(BRC_CTR_ASP_LIN_Ndx);
	//Getting the index of lin rx/tx buffer
	pBuf = (uint8_t *)LIN_FRAME_GET_BUF(BRC_CTR_ASP_LIN_Ndx);
	//TODO: This memcpy can be removed in future	
	memcpy(LinReceivedData, pBuf,  u8Len);
	//TODO: Function is not implemented properly.Can be removed in future. Implemented to avoid interrupts during msg copy operation 
	InterruptsSuspend();
	memcpy(&l_pFrameBuf[4],pBuf, 8);
	//TODO: Function is not implemented properly.
	InterruptsRestore();
	//TODO: This part can be removed 
	/*	  l_pFrameBuf[4] = LinReceivedData[0];
          l_pFrameBuf[5] = LinReceivedData[1];
          l_pFrameBuf[6] = LinReceivedData[2];
          l_pFrameBuf[7] = LinReceivedData[3];
          l_pFrameBuf[8] = LinReceivedData[4];
          l_pFrameBuf[9] = LinReceivedData[5];
          l_pFrameBuf[10] = LinReceivedData[6];
          l_pFrameBuf[11] = LinReceivedData[7];	*/
		 
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  BRC_MEM_POS_EXMI_LIN_Notification(void)// Message direction master to slave
{
	uint8_t u8Len,*pBuf;
	u8Len =  LIN_FRAME_GET_BUF_LEN(BRC_MEM_POS_EXMI_LIN_Ndx);
	pBuf = (uint8_t *)LIN_FRAME_GET_BUF(BRC_MEM_POS_EXMI_LIN_Ndx);
	memset(	LinReceivedData,0,8);
	memcpy(LinReceivedData, pBuf,  u8Len); 
	/* frame BRC_MEM_POS_EXMI_LIN */
    l_pFrameBuf[0] = LinReceivedData[0];
    l_pFrameBuf[1] = LinReceivedData[1];
    l_pFrameBuf[2] = LinReceivedData[2];
    l_pFrameBuf[3] = LinReceivedData[3];

    /* set the rx flags for the frame */
    l_pRxFlagData[0] = 0xFF;
    /* set the changed flags for the frame */
    LIN_SET_CHANGED_FLAG_BYTE(0, (l_u8) 0xFF);

        
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/

void  ST_POS_EXMI_LH_LIN_Notification(void)	 // Message direction master to slave
{
	static uint8_t au8TxArr[2];
	au8TxArr[0]	= l_pFrameBuf[22];
	au8TxArr[1] = l_pFrameBuf[23];
	LINCom_SendMessage(ST_POS_EXMI_LH_LIN,au8TxArr);
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  ST_POS_EXMI_RH_LIN_Notification(void)	 // Message direction master to slave
{
	static uint8_t au8TxArr[2];
	au8TxArr[0]	= l_pFrameBuf[22];
	au8TxArr[1] = l_pFrameBuf[23];
	LINCom_SendMessage(ST_POS_EXMI_RH_LIN,au8TxArr);
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  ST_HC2_ASP_RH_LIN_Notification(void)	 // Message direction master to slave
{
	static uint8_t au8TxArr[2];
	au8TxArr[0]	= l_pFrameBuf[20];
	au8TxArr[1] = l_pFrameBuf[21];
	LINCom_SendMessage(ST_HC2_ASP_LH_LIN,au8TxArr);
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  ST_HC2_ASP_LH_LIN_Notification(void)	// Message direction slave to master
{
	static uint8_t au8TxArr[2];
	au8TxArr[0]	= l_pFrameBuf[20];
	au8TxArr[1] = l_pFrameBuf[21];
	LINCom_SendMessage(ST_HC2_ASP_LH_LIN,au8TxArr);
	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  ERR_ST_ASP_LH_LIN_Notification(void)	 // Message direction master to slave
{
	  static uint8_t au8TxArr[8];
	  au8TxArr[0] = l_pFrameBuf[12];
      au8TxArr[1] = l_pFrameBuf[13];
      au8TxArr[2] = l_pFrameBuf[14];
      au8TxArr[3] = l_pFrameBuf[15];
      au8TxArr[4] = l_pFrameBuf[16];
      au8TxArr[5] = l_pFrameBuf[17];
      au8TxArr[6] = l_pFrameBuf[18];
      au8TxArr[7] = l_pFrameBuf[19];
	  LINCom_SendMessage(ERR_ST_ASP_LH_LIN,au8TxArr);
}


/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  ERR_ST_ASP_RH_LIN_Notification(void)	 // Message direction master to slave
{
	  static uint8_t au8TxArr[8];
	  au8TxArr[0] = l_pFrameBuf[12];
      au8TxArr[1] = l_pFrameBuf[13];
      au8TxArr[2] = l_pFrameBuf[14];
      au8TxArr[3] = l_pFrameBuf[15];
      au8TxArr[4] = l_pFrameBuf[16];
      au8TxArr[5] = l_pFrameBuf[17];
      au8TxArr[6] = l_pFrameBuf[18];
      au8TxArr[7] = l_pFrameBuf[19];
	  LINCom_SendMessage(ERR_ST_ASP_LH_LIN,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  MasterReq_Notification(void)	 // Message direction slave to master
{
		  
	  uint8_t u8Len,*pBuf;
	  u8Len =  LIN_FRAME_GET_BUF_LEN(BRC_MEM_POS_EXMI_LIN_Ndx);
      pBuf = LIN_FRAME_GET_BUF(BRC_MEM_POS_EXMI_LIN_Ndx);
	  memset(	LinReceivedData,0,8);
  	  memcpy(LinReceivedData, pBuf,  u8Len); 
	
	  l_pFrameBuf[96] = LinReceivedData[0];
      l_pFrameBuf[97] = LinReceivedData[1];
      l_pFrameBuf[98] = LinReceivedData[2];
      l_pFrameBuf[99] = LinReceivedData[3];
      l_pFrameBuf[100] = LinReceivedData[4];
      l_pFrameBuf[101] = LinReceivedData[5];
      l_pFrameBuf[102] = LinReceivedData[6];
      l_pFrameBuf[103] = LinReceivedData[7];
      /* set the rx flags for the frame */
      l_pRxFlagData[34] = 0xFF;
      l_pRxFlagData[35] = 0xFF;
      /* set the changed flags for the frame */
      LIN_SET_CHANGED_FLAG_BYTE(33, (l_u8) 0xFF);
 }
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  SlaveResp_Notification(void)	 // Message direction slave to master
{
 	  static uint8_t au8TxArr[8];
	  au8TxArr[0] = l_pFrameBuf[32];
      au8TxArr[1] = l_pFrameBuf[33];
      au8TxArr[2] = l_pFrameBuf[34];
      au8TxArr[3] = l_pFrameBuf[35];
      au8TxArr[4] = l_pFrameBuf[36];
      au8TxArr[5] = l_pFrameBuf[37];
      au8TxArr[6] = l_pFrameBuf[38];
      au8TxArr[7] = l_pFrameBuf[39];
	  LINCom_SendMessage(SlaveResp,au8TxArr);
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/

void  DBG_ASP_LH_ZAMC_CTRL_Notification(void)	 // Message direction slave to master  ERR_ST_ASP_LH_LIN
{
	  static uint8_t au8TxArr[8];
	  /* au8TxArr[0] = l_pFrameBuf[32];
      au8TxArr[1] = l_pFrameBuf[33];
      au8TxArr[2] = l_pFrameBuf[34];
      au8TxArr[3] = l_pFrameBuf[35];
      au8TxArr[4] = l_pFrameBuf[36];
      au8TxArr[5] = l_pFrameBuf[37];
      au8TxArr[6] = l_pFrameBuf[38];
      au8TxArr[7] = l_pFrameBuf[39];*/
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[33], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_LH_ZAMC_CTRL,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  DBG_ASP_RH_ZAMC_CTRL_Notification(void)	 // Message direction slave to master  
{
	 static uint8_t au8TxArr[8];
	  /* au8TxArr[0] = l_pFrameBuf[40];
      au8TxArr[1] = l_pFrameBuf[41];
      au8TxArr[2] = l_pFrameBuf[42];
      au8TxArr[3] = l_pFrameBuf[43];
      au8TxArr[4] = l_pFrameBuf[44];
      au8TxArr[5] = l_pFrameBuf[45];
      au8TxArr[6] = l_pFrameBuf[46];
      au8TxArr[7] = l_pFrameBuf[47];  */
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[40], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_RH_ZAMC_CTRL,au8TxArr);	
}
void  DBG_ASP_LH_ZAMC_STAT_Notification(void)	 // Message direction slave to master  ERR_ST_ASP_LH_LIN
{
	  static uint8_t au8TxArr[8];
	  /* au8TxArr[0] = l_pFrameBuf[40];
      au8TxArr[1] = l_pFrameBuf[41];
      au8TxArr[2] = l_pFrameBuf[42];
      au8TxArr[3] = l_pFrameBuf[43];
      au8TxArr[4] = l_pFrameBuf[44];
      au8TxArr[5] = l_pFrameBuf[45];
      au8TxArr[6] = l_pFrameBuf[46];
      au8TxArr[7] = l_pFrameBuf[47]; */
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[40], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_LH_ZAMC_STAT,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  DBG_ASP_RH_ZAMC_STAT_Notification(void)	 // Message direction slave to master  
{
	 static uint8_t au8TxArr[8];
	  /* au8TxArr[0] = l_pFrameBuf[32];
      au8TxArr[1] = l_pFrameBuf[33];
      au8TxArr[2] = l_pFrameBuf[34];
      au8TxArr[3] = l_pFrameBuf[35];
      au8TxArr[4] = l_pFrameBuf[36];
      au8TxArr[5] = l_pFrameBuf[37];
      au8TxArr[6] = l_pFrameBuf[38];
      au8TxArr[7] = l_pFrameBuf[39];*/
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[32], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_RH_ZAMC_STAT,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/ 
////////////////////////////////   LH
void  DBG_ASP_LH_STAT1_Notification(void)	 // Message direction slave to master  ERR_ST_ASP_LH_LIN
{
	  static uint8_t au8TxArr[8];
	  /* au8TxArr[0] = l_pFrameBuf[48];
      au8TxArr[1] = l_pFrameBuf[49];
      au8TxArr[2] = l_pFrameBuf[50];
      au8TxArr[3] = l_pFrameBuf[51];
      au8TxArr[4] = l_pFrameBuf[52];
      au8TxArr[5] = l_pFrameBuf[53];
      au8TxArr[6] = l_pFrameBuf[54];
      au8TxArr[7] = l_pFrameBuf[55];*/
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[48], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_LH_STAT1,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  DBG_ASP_LH_STAT2_Notification(void)	 // Message direction slave to master  
{
	  static uint8_t au8TxArr[8];
	  /*  au8TxArr[0] = l_pFrameBuf[56];
      au8TxArr[1] = l_pFrameBuf[57];
      au8TxArr[2] = l_pFrameBuf[58];
      au8TxArr[3] = l_pFrameBuf[59];
      au8TxArr[4] = l_pFrameBuf[60];
      au8TxArr[5] = l_pFrameBuf[61];
      au8TxArr[6] = l_pFrameBuf[62];
      au8TxArr[7] = l_pFrameBuf[63]; */
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[56], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_LH_STAT2,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  DBG_ASP_LH_STAT3_Notification(void)	 // Message direction slave to master  
{
	  static uint8_t au8TxArr[8];
	  /*  au8TxArr[0] = l_pFrameBuf[64];
      au8TxArr[1] = l_pFrameBuf[65];
      au8TxArr[2] = l_pFrameBuf[66];
      au8TxArr[3] = l_pFrameBuf[67];
      au8TxArr[4] = l_pFrameBuf[68];
      au8TxArr[5] = l_pFrameBuf[69];
      au8TxArr[6] = l_pFrameBuf[70];
      au8TxArr[7] = l_pFrameBuf[71]; */
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[64], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_LH_STAT3,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  DBG_ASP_LH_STAT4_Notification(void)	 // Message direction slave to master  
{
	  static uint8_t au8TxArr[8];
	   /* au8TxArr[0] = l_pFrameBuf[72];
      au8TxArr[1] = l_pFrameBuf[73];
      au8TxArr[2] = l_pFrameBuf[74];
      au8TxArr[3] = l_pFrameBuf[75];
      au8TxArr[4] = l_pFrameBuf[76];
      au8TxArr[5] = l_pFrameBuf[77];
      au8TxArr[6] = l_pFrameBuf[78];
      au8TxArr[7] = l_pFrameBuf[79]; */
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[72], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_LH_STAT4,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  DBG_ASP_LH_STAT5_Notification(void)	 // Message direction slave to master  
{
	  static uint8_t au8TxArr[8];
	  /*  au8TxArr[0] = l_pFrameBuf[80];
      au8TxArr[1] = l_pFrameBuf[81];
      au8TxArr[2] = l_pFrameBuf[82];
      au8TxArr[3] = l_pFrameBuf[83];
      au8TxArr[4] = l_pFrameBuf[84];
      au8TxArr[5] = l_pFrameBuf[85];
      au8TxArr[6] = l_pFrameBuf[86];
      au8TxArr[7] = l_pFrameBuf[87];*/
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[80], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_LH_STAT5,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  DBG_ASP_LH_STAT6_Notification(void)	 // Message direction slave to master  
{
	  static uint8_t au8TxArr[8];
	  /*  au8TxArr[0] = l_pFrameBuf[88];
      au8TxArr[1] = l_pFrameBuf[89];
      au8TxArr[2] = l_pFrameBuf[90];
      au8TxArr[3] = l_pFrameBuf[91];
      au8TxArr[4] = l_pFrameBuf[92];
      au8TxArr[5] = l_pFrameBuf[93];
      au8TxArr[6] = l_pFrameBuf[94];
      au8TxArr[7] = l_pFrameBuf[95]; */
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[88], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_LH_STAT6,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
////////////////////////////////   RH
void  DBG_ASP_RH_STAT1_Notification(void)	 // Message direction slave to master  ERR_ST_ASP_RH_LIN
{
	  static uint8_t au8TxArr[8];
	   /* au8TxArr[0] = l_pFrameBuf[48];
      au8TxArr[1] = l_pFrameBuf[49];
      au8TxArr[2] = l_pFrameBuf[50];
      au8TxArr[3] = l_pFrameBuf[51];
      au8TxArr[4] = l_pFrameBuf[52];
      au8TxArr[5] = l_pFrameBuf[53];
      au8TxArr[6] = l_pFrameBuf[54];
      au8TxArr[7] = l_pFrameBuf[55]; */
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[48], 8);
	  InterruptsRestore();

	  LINCom_SendMessage(DBG_ASP_RH_STAT1,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  DBG_ASP_RH_STAT2_Notification(void)	 // Message direction slave to master  
{
	  static uint8_t au8TxArr[8];
	   /* au8TxArr[0] = l_pFrameBuf[56];
      au8TxArr[1] = l_pFrameBuf[57];
      au8TxArr[2] = l_pFrameBuf[58];
      au8TxArr[3] = l_pFrameBuf[59];
      au8TxArr[4] = l_pFrameBuf[60];
      au8TxArr[5] = l_pFrameBuf[61];
      au8TxArr[6] = l_pFrameBuf[62];
      au8TxArr[7] = l_pFrameBuf[63]; */
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[56], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_RH_STAT2,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  DBG_ASP_RH_STAT3_Notification(void)	 // Message direction slave to master  
{
	  static uint8_t au8TxArr[8];
	   /* au8TxArr[0] = l_pFrameBuf[64];
      au8TxArr[1] = l_pFrameBuf[65];
      au8TxArr[2] = l_pFrameBuf[66];
      au8TxArr[3] = l_pFrameBuf[67];
      au8TxArr[4] = l_pFrameBuf[68];
      au8TxArr[5] = l_pFrameBuf[69];
      au8TxArr[6] = l_pFrameBuf[70];
      au8TxArr[7] = l_pFrameBuf[71];*/
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[64], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_RH_STAT3,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  DBG_ASP_RH_STAT4_Notification(void)	 // Message direction slave to master  
{
	 static uint8_t au8TxArr[8];
	 /*  au8TxArr[0] = l_pFrameBuf[72];
      au8TxArr[1] = l_pFrameBuf[73];
      au8TxArr[2] = l_pFrameBuf[74];
      au8TxArr[3] = l_pFrameBuf[75];
      au8TxArr[4] = l_pFrameBuf[76];
      au8TxArr[5] = l_pFrameBuf[77];
      au8TxArr[6] = l_pFrameBuf[78];
      au8TxArr[7] = l_pFrameBuf[79];*/
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[72], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_RH_STAT4,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  DBG_ASP_RH_STAT5_Notification(void)	 // Message direction slave to master  
{
	 static uint8_t au8TxArr[8];
	  /* au8TxArr[0] = l_pFrameBuf[80];
      au8TxArr[1] = l_pFrameBuf[81];
      au8TxArr[2] = l_pFrameBuf[82];
      au8TxArr[3] = l_pFrameBuf[83];
      au8TxArr[4] = l_pFrameBuf[84];
      au8TxArr[5] = l_pFrameBuf[85];
      au8TxArr[6] = l_pFrameBuf[86];
      au8TxArr[7] = l_pFrameBuf[87];*/
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[80], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_RH_STAT5,au8TxArr);	
}
/**
  * @brief  This function copies LIN messages to array   
  * @param  None
  * @retval None
*/
void  DBG_ASP_RH_STAT6_Notification(void)	 // Message direction slave to master  
{
	 static uint8_t au8TxArr[8];
	  /* au8TxArr[0] = l_pFrameBuf[88];
      au8TxArr[1] = l_pFrameBuf[89];
      au8TxArr[2] = l_pFrameBuf[90];
      au8TxArr[3] = l_pFrameBuf[91];
      au8TxArr[4] = l_pFrameBuf[92];
      au8TxArr[5] = l_pFrameBuf[93];
      au8TxArr[6] = l_pFrameBuf[94];
      au8TxArr[7] = l_pFrameBuf[95];*/
	  InterruptsSuspend();
	  memcpy(&au8TxArr,&l_pFrameBuf[88], 8);
	  InterruptsRestore();
	  LINCom_SendMessage(DBG_ASP_RH_STAT6,au8TxArr);	
} 



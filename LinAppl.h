/**********************************************************************************************************************************
Project     :  L7-Mirror
Filename    :  LinAppl.h
Brief descr.:  Headerfile for LIN communication, application specific part with signal oriented interface.
               Declaration of signal variables and functions used by the application.
               Signal names are according to LIN spec and BMW communication matrix.
Copyright   :  
Remarks     :  
Version
  1.0.0  18.09.2009  UF  Creation
**********************************************************************************************************************************/
#ifndef _LINAPPL_H_
#define _LINAPPL_H_

/* Include-Files ################################################################################################################*/
/* Defines ######################################################################################################################*/

// Bit Index used for function l_u8_wr_ERR_ST_EXMI_XH_LIN
#define LIN_ERR_IDX_WATCHDOG      0
#define LIN_ERR_IDX_HEATER        1
#define LIN_ERR_IDX_EC_GLASS      2
#define LIN_ERR_IDX_MPOT          3
#define LIN_ERR_IDX_MMOT          4
#define LIN_ERR_IDX_HC2           5
#define LIN_ERR_IDX_PFLD          9
#define LIN_ERR_IDX_UNDERVOLTAGE 11
#define LIN_ERR_IDX_OVERVOLTAGE	 12
#define LIN_ERR_IDX_UV_HEATER    13
#define LIN_ERR_IDX_OVERTEMP	   14

#ifdef _LEFT_MIRROR
# define l_u8_rd_CTR_DSE_1_XH_WARN_LNCH_LIN   l_u8_rd_CTR_DSE_1_LH_WARN_LNCH_LIN

# define l_u8_wr_PO_AVL_EXMI_XH_AX_RH_LH_LIN  l_u8_wr_PO_AVL_EXMI_LH_AX_RH_LH_LIN
# define l_u8_rd_PO_AVL_EXMI_XH_AX_RH_LH_LIN  l_u8_rd_PO_AVL_EXMI_LH_AX_RH_LH_LIN
# define l_u8_wr_PO_AVL_EXMI_XH_AX_UP_DWN_LIN l_u8_wr_PO_AVL_EXMI_LH_AX_DWN_UP_LIN
# define l_u8_rd_PO_AVL_EXMI_XH_AX_UP_DWN_LIN l_u8_rd_PO_AVL_EXMI_LH_AX_DWN_UP_LIN

# define LIN_BYTE_OFFSET_ERR_ST_EXMI_XH_00_LIN LIN_BYTE_OFFSET_ERR_ST_EXMI_LH_00_LIN
# define l_u8_rd_ERR_ST_EXMI_XH_LIN(idx)      ((l_pFrameBuf[LIN_BYTE_OFFSET_ERR_ST_EXMI_LH_00_LIN+idx/4] >> ((idx & 0x03)*2))& 0x03)

# define l_u8_wr_ST_DSE_1_XH_WARN_LNCH_LIN    l_u8_wr_ST_DSE_1_LH_WARN_LNCH_LIN
# define l_u8_wr_NOD_ERR_EXMI_XH_LIN          l_u8_wr_NOD_ERR_EXMI_LH_LIN
# define l_flg_clr_TxNOD_ERR_EXMI_XH_LIN      l_flg_clr_TxNOD_ERR_EXMI_LH_LIN
# define l_flg_tst_TxNOD_ERR_EXMI_XH_LIN      l_flg_tst_TxNOD_ERR_EXMI_LH_LIN
# define l_bool_wr_COMM_ERR_EXMI_XH_LIN       l_bool_wr_COMM_ERR_EXMI_LH_LIN
# define l_u8_wr_VRS_EXMI_XH_LIN              l_u8_wr_VRS_EXMI_LH_LIN

# define LIN_BYTE_OFFSET_DBG_ASP_XH_L99_CR1_WDTRIG     LIN_BYTE_OFFSET_DBG_ASP_LH_L99_CR1_WDTRIG
# define LIN_BYTE_OFFSET_DBG_ASP_XH_L99_SR1_OUT1_LS_OC LIN_BYTE_OFFSET_DBG_ASP_LH_L99_SR1_OUT1_LS_OC
# define LIN_BYTE_OFFSET_DBG_ASP_XH_TMON_ADC           LIN_BYTE_OFFSET_DBG_ASP_LH_TMON_ADC
# define LIN_BYTE_OFFSET_DBG_ASP_XH_EC_DIAG_OUT        LIN_BYTE_OFFSET_DBG_ASP_LH_EC_DIAG_OUT
# define LIN_BYTE_OFFSET_DBG_ASP_XH_SW_VERSION_SUB     LIN_BYTE_OFFSET_DBG_ASP_LH_SW_VERSION_SUB
# define LIN_BYTE_OFFSET_DBG_ASP_XH_PWM_PERIOD         LIN_BYTE_OFFSET_DBG_ASP_LH_PWM_PERIOD
# define LIN_BYTE_OFFSET_DBG_ASP_XH_HC2_BRIGHTNESS     LIN_BYTE_OFFSET_DBG_ASP_LH_HC2_BRIGHTNESS
# define LIN_BYTE_OFFSET_DBG_ASP_XH_HB_ADC             LIN_BYTE_OFFSET_DBG_ASP_LH_HB_ADC
# define l_u8_wr_DBG_ASP_XH_SW_VERSION_SUB    l_u8_wr_DBG_ASP_LH_SW_VERSION_SUB
#endif

#ifdef _RIGHT_MIRROR
# define l_u8_rd_CTR_DSE_1_XH_WARN_LNCH_LIN   l_u8_rd_CTR_DSE_1_RH_WARN_LNCH_LIN
	 
# define l_u8_wr_PO_AVL_EXMI_XH_AX_RH_LH_LIN  l_u8_wr_PO_AVL_EXMI_RH_AX_RH_LH_LIN
# define l_u8_rd_PO_AVL_EXMI_XH_AX_RH_LH_LIN  l_u8_rd_PO_AVL_EXMI_RH_AX_RH_LH_LIN
# define l_u8_wr_PO_AVL_EXMI_XH_AX_UP_DWN_LIN l_u8_wr_PO_AVL_EXMI_RH_AX_UP_DWN_LIN
# define l_u8_rd_PO_AVL_EXMI_XH_AX_UP_DWN_LIN l_u8_rd_PO_AVL_EXMI_RH_AX_UP_DWN_LIN

# define LIN_BYTE_OFFSET_ERR_ST_EXMI_XH_00_LIN LIN_BYTE_OFFSET_ERR_ST_EXMI_RH_00_LIN
# define l_u8_rd_ERR_ST_EXMI_XH_LIN(idx)      ((l_pFrameBuf[LIN_BYTE_OFFSET_ERR_ST_EXMI_RH_00_LIN+idx/4] >> ((idx & 0x03)*2))& 0x03)

# define l_u8_wr_ST_DSE_1_XH_WARN_LNCH_LIN    l_u8_wr_ST_DSE_1_RH_WARN_LNCH_LIN
# define l_u8_wr_NOD_ERR_EXMI_XH_LIN          l_u8_wr_NOD_ERR_EXMI_RH_LIN
# define l_flg_clr_TxNOD_ERR_EXMI_XH_LIN      l_flg_clr_TxNOD_ERR_EXMI_RH_LIN
# define l_flg_tst_TxNOD_ERR_EXMI_XH_LIN      l_flg_tst_TxNOD_ERR_EXMI_RH_LIN
# define l_bool_wr_COMM_ERR_EXMI_XH_LIN       l_bool_wr_COMM_ERR_EXMI_RH_LIN
# define l_u8_wr_VRS_EXMI_XH_LIN              l_u8_wr_VRS_EXMI_RH_LIN

# define LIN_BYTE_OFFSET_DBG_ASP_XH_L99_CR1_WDTRIG     LIN_BYTE_OFFSET_DBG_ASP_RH_L99_CR1_WDTRIG
# define LIN_BYTE_OFFSET_DBG_ASP_XH_L99_SR1_OUT1_LS_OC LIN_BYTE_OFFSET_DBG_ASP_RH_L99_SR1_OUT1_LS_OC
# define LIN_BYTE_OFFSET_DBG_ASP_XH_TMON_ADC           LIN_BYTE_OFFSET_DBG_ASP_RH_TMON_ADC
# define LIN_BYTE_OFFSET_DBG_ASP_XH_EC_DIAG_OUT        LIN_BYTE_OFFSET_DBG_ASP_RH_EC_DIAG_OUT
# define LIN_BYTE_OFFSET_DBG_ASP_XH_SW_VERSION_SUB     LIN_BYTE_OFFSET_DBG_ASP_RH_SW_VERSION_SUB
# define LIN_BYTE_OFFSET_DBG_ASP_XH_PWM_PERIOD         LIN_BYTE_OFFSET_DBG_ASP_RH_PWM_PERIOD
# define LIN_BYTE_OFFSET_DBG_ASP_XH_HC2_BRIGHTNESS     LIN_BYTE_OFFSET_DBG_ASP_RH_HC2_BRIGHTNESS
# define LIN_BYTE_OFFSET_DBG_ASP_XH_HB_ADC             LIN_BYTE_OFFSET_DBG_ASP_RH_HB_ADC
# define l_u8_wr_DBG_ASP_XH_SW_VERSION_SUB    l_u8_wr_DBG_ASP_RH_SW_VERSION_SUB
#endif

// Values for CTR_EXMI_MEMO_LIN
#define LIN_MPOT_SELECT_ASP_LEFT    0
#define LIN_MPOT_SELECT_ASP_RIGHT   1
#define LIN_MPOT_SELECT_ASP_SNV     3

// Values for BRC_CTR_EXMI_LIN
#define LIN_CTRL_SELECT_ASP_LEFT    0
#define LIN_CTRL_SELECT_ASP_RIGHT   1
#define LIN_CTRL_SELECT_ASP_BOTH    2
#define LIN_CTRL_SELECT_ASP_SNV     3

// Values for BRC_CTR_EXMI_AD_LIN
#define LIN_CTRL_MMOT_ADJUST_NO     0
#define LIN_CTRL_MMOT_ADJUST_LEFT   1
#define LIN_CTRL_MMOT_ADJUST_RIGHT  2
#define LIN_CTRL_MMOT_ADJUST_UP     4
#define LIN_CTRL_MMOT_ADJUST_DOWN   8
#define LIN_CTRL_MMOT_ADJUST_SNV   15

// Values for BRC_CTR_EXMI_FMIR_LIN
#define LIN_CTRL_PFLD_NO    0
#define LIN_CTRL_PFLD_DRIVE 2
#define LIN_CTRL_PFLD_PARK  1
#define LIN_CTRL_PFLD_SNV   3

// Values for BRC_CTR_HT_EXMI
#define LIN_CTRL_HEATER_OFF 0
#define LIN_CTRL_HEATER_25  1
#define LIN_CTRL_HEATER_50  2
#define LIN_CTRL_HEATER_75  3
#define LIN_CTRL_HEATER_100 4 
#define LIN_CTRL_HEATER_SNV 7

// Values for CTR_DIM_DSE_LNCH_LIN
#define LIN_CTRL_HC2_DIMMING_MIN  0
//#define LIN_CTRL_HC2_DIMMING_MAX  254


#if HC2_BRIGHTNESS==LINEAR              // for linear brightness
#define LIN_CTRL_HC2_DIMMING_MAX  1502   
#else if HC2_BRIGHTNESS==QUADRATIC      // for quadratic brightness
#define LIN_CTRL_HC2_DIMMING_MAX  1613   
#endif

#define LIN_CTRL_HC2_DIMMING_SNV  255

// Values for LCTR_DSE_1_LH_WARN_LNCH_LIN/CTR_DSE_1_RH_WARN_LNCH_LIN
#define LIN_CTRL_HC2_MODE_OFF   0
#define LIN_CTRL_HC2_MODE_ON    1
#define LIN_CTRL_HC2_MODE_BL1   2
#define LIN_CTRL_HC2_MODE_BL2   3
#define LIN_CTRL_HC2_MODE_BL3   4
#define LIN_CTRL_HC2_MODE_INIT  8 // Not used - purpose unknown, treated as "invalid"
#define LIN_CTRL_HC2_MODE_SNV  15


// Values for ERR_ST_EXMI_LH_00_LIN..ERR_ST_EXMI_LH_15_LIN/ERR_ST_EXMI_RH_00_LIN..ERR_ST_EXMI_RH_15_LIN)
#define LIN_ERR_OK  0 
#define LIN_ERR_ERR 1
#define LIN_ERR_SNV 3

// Values for ST_DSE_1_LH_WARN_LNCH_LIN/ST_DSE_1_RH_WARN_LNCH_LIN
#define LIN_HC2_FEEDBACK_OFF   0
#define LIN_HC2_FEEDBACK_ON    1
#define LIN_HC2_FEEDBACK_BL1   2
#define LIN_HC2_FEEDBACK_BL2   3
#define LIN_HC2_FEEDBACK_BL3   4
#define LIN_HC2_FEEDBACK_ERR  13
#define LIN_HC2_FEEDBACK_SNV  15

// Values for NOD_ERR_EXMI_LH_LIN/NOD_ERR_EXMI_RH_LIN
#define LIN_NOD_ERR_NO        0
#define LIN_NOD_ERR_ERR       1
#define LIN_NOD_ERR_CHANGE    2
#define LIN_NOD_ERR_SNV       3

// Values for COMM_ERR_EXMI_LH_LIN/COMM_ERR_EXMI_RH_LIN
#define LIN_COMM_ERR_NO  0
#define LIN_COMM_ERR_ERR 1

// All signals with memory potentiometer values
#define LIN_POTI_SNV 255

/* Type-definitions #############################################################################################################*/
/* Declarations #################################################################################################################*/
#ifdef _LINAPPL_C_
# define extern
#endif
/* Global variables #############################################################################################################*/

/* Interface-functions ##########################################################################################################*/
extern void LinApplInit( void);
extern void LinApplUpdateBrcCtrlAsp( void);
extern Std_ReturnType LinApplUpdateBrcMemPos( void);
extern void l_u8_wr_ERR_ST_EXMI_XH_LIN( uint8 err_idx, uint8 new_val);
extern void LinApplUpdateTxBuffers( void);

#ifdef extern
# undef extern
#endif
#endif // _xxx_H_
/*-- End of File --*/

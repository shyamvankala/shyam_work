    #include "stdint.h"
#include "std_init.h"
#include "ZMDCM0.h"
#include "SPI.h"
#include "ADC.h"
  #include "std_init.h"
#include "stdint.h"

#include "ZMDCM0.h"

#include "SPI.h"

#include "SBC.h"

#include "ECM_cfg.h"

#include "ECM.h"
#include "LINCom.h"
#include "SBC_cfg.h"
 #include "LINCom_cfg.h"
typedef uint8_t Std_ReturnType;

#define _DEBUG	ON
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
/*
#if HC2_BRIGHTNESS==LINEAR              // for linear brightness
#define LIN_CTRL_HC2_DIMMING_MAX  1502   
#else if HC2_BRIGHTNESS==QUADRATIC      // for quadratic brightness
#define LIN_CTRL_HC2_DIMMING_MAX  1613   
#endif
*/
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

// #define AdcRead() AdcReadCurrentMonitor()
#define HTR_CM 	eNdxAIN_HS1 // eHS1_MUX_NDX	  eNdxAIN_HS1
#define	EC_VM	eNdxAIN_ECM
#define HC2_CM  eNdxAIN_HS2
#define ADC_CH_UBAT      eNdxAIN_VDDE		   //for ADC VOLTAGE 
// not to be changed by application!
/**
 * Controls the ECM output 
 * \param  u8ECMValue The new value of the ECMDAC in ECMCTROL registers.  
 * \return none
 *
 * \brief This function  sets u8ECMValue into ECMCTRL register. The parameter should be in range 0-63. After this function is called the update of ECMCTRL register is triggered.
 * <br>Possible values are :<br> 
 * #ECM_OFF,#ECM_MAX,#ECM_0,#ECM_1,#ECM_2,#ECM_3,#ECM_4,#ECM_5,#ECM_6,#ECM_7,#ECM_8,#ECM_9,#ECM_10,#ECM_11,
 * #ECM_12,#ECM_13,#ECM_14,#ECM_15,#ECM_16,#ECM_17,#ECM_18,#ECM_19,#ECM_20,#ECM_21,#ECM_22,#ECM_23,#ECM_24,#ECM_25,
 * #ECM_26,#ECM_27,#ECM_28,#ECM_29,#ECM_30,#ECM_31,#ECM_32,#ECM_33,#ECM_34,#ECM_35,#ECM_36,#ECM_37,#ECM_38,#ECM_39,
 * #ECM_40,#ECM_41,#ECM_42,#ECM_43,#ECM_44,#ECM_45,#ECM_46,#ECM_47,#ECM_48,#ECM_49,#ECM_50,#ECM_51,#ECM_52,#ECM_53,
 * #ECM_54,#ECM_55,#ECM_56,#ECM_57,#ECM_58,#ECM_59,#ECM_60,#ECM_61,#ECM_62,#ECM_63
 */
#define EC_CONTROL_OFF_DAC_MIN()   ECM_u8SetECMValue(3);//gL99CtrlReg2.sig.EC=3;gL99CtrlReg2.sig.ECVLS=0;
#define EC_CONTROL_OFF()           ECM_u8SetECMValue(0);//gL99CtrlReg2.sig.EC=1;gL99CtrlReg2.sig.ECVLS=0;
#define EC_CONTROL_CLEAR()         ECM_u8SetECMValue(0);//gL99CtrlReg2.sig.EC=1;gL99CtrlReg2.sig.ECVLS=1;
#define EC_CONTROL(val)            ECM_u8SetECMValue(val);//((val)<<1)+1);//;gL99CtrlReg2.sig.ECVLS=0;
//#define EC_VOLTAGE_STATUS_LOW()    0//gL99StatReg3.sig.ECV_MONI_LOW
//#define EC_VOLTAGE_STATUS_HIGH()   0//gL99StatReg3.sig.ECV_MONI_HIGH
#define EC_HS_OVERCURRENT()        EC_HS_LS_OVERCURRENT()//0//gL99StatReg1.sig.OUT7_OC
#define EC_HS_OPEN_LOAD()          0//gL99StatReg2.sig.OUT7_UC
#define EC_CM_RAW( pCurr)          0//L99ApplReadCurrentMonitorRaw(L99_OUT7, pCurr)
#define EC_ICM_AMPS_TO_LSBS        0//ICM_AMPS_TO_LSBS_OUT7
#define EC_LS_OVERCURRENT()        EC_HS_LS_OVERCURRENT()//EC_OC_FLAG//0//EC_HS_LS_OVERCURRENT()//0//gL99StatReg1.sig.ECV_OC
#define EC_LS_OPEN_LOAD()          0//gL99StatReg2.sig.ECV_UC



 #define HTR_CM_RAW( pCurr)        ApplReadCurrentMonitorRaw(HTR_CM, pCurr)
 #define HC2_CM_RAW(pCurr)		   ApplReadCurrentMonitorRaw(HC2_CM, pCurr)
//#define MMOT_OVERCURRENT()       1 //((gL99StatReg1.word&(L99_HBR_IN_MASK(MMOT_LEFT)|L99_HBR_IN_MASK(MMOT_RIGHT)|L99_HBR_IN_MASK(MMOT_UP)|L99_HBR_IN_MASK(MMOT_DOWN)))!=0)
//#define MMOT_OPEN_LOAD( mask)    1 // ((gL99StatReg2.word&(mask))==(mask))


  #define RESCODE 1024/350000
  #define RESCODEHB3_4 1024/75000
// #define AdcRead() AdcReadCurrentMonitor()

#define PFLD_CM_drive eNdxAIN_HB1H
#define PFLD_CM_park eNdxAIN_HB2H

#define PFLD_CM_RAW_pfld(MuxVal,pCurr)	  ((MuxVal == PFLD_DIR_DRIVE)?ApplReadCurrentMonitorRaw_pfld(PFLD_CM_drive, pCurr) :  ApplReadCurrentMonitorRaw_pfld(PFLD_CM_park, pCurr)) 
                                              
											  										  											 
											 
											  
											  
  //#define PFLD_CM_RAW_p(pCurr)	   ApplReadCurrentMonitorRaw_park(PFLD_CM_park, pCurr)
 
#define IREF_NOM 110/100

#define I_CM_MEAS_MA_PER_LSB_LOW_RDS_ON   ((const float)(((IREF_NOM)*1000)*(RESCODE )))
// For powerfold no problem. Only Rds Low is used in any direction
#define PFLD_ICM_AMPS_TO_LSBS(val) ((const uint16)(((val)*1000.)/I_CM_MEAS_MA_PER_LSB_LOW_RDS_ON))

void ZMDIApplInit(void);
 extern void SBC_ApplUpdateCurrentmonitor(uint8_t MuxVal);
 Std_ReturnType ApplReadCurrentMonitorRaw_pfld( uint8_t MuxVal, uint16_t* pCurrent);
 //Std_ReturnType ApplReadCurrentMonitorRaw_(uint8_t MuxVal, uint16_t* pCurrent);
 Std_ReturnType ApplReadCurrentMonitorRaw(uint8_t MuxVal, uint16_t* pCurrent);
 extern void PFLD_MOVE_DRIVE(void);
 extern void PFLD_MOVE_PARK(void);
 extern void PFLD_BRAKE_PARK(void); 
extern void PFLD_BRAKE_DRIVE(void);
 void LinApplUpdateBrcCtrlAsp( void);
 void ZMDIApplBeforeWriteOutputs( void);
 void ZMDIApplBeforeWriteOutputs( void);
extern void PWM_vSettings (uint16_t u16Freq, uint8_t u8DCycle);
Std_ReturnType ZMDIApplReadCurrentMonitorRaw( uint8_t MuxVal, uint16_t* pCurrent);
uint8_t CTRL_Reg_Read(uint8_t value);
//uint8_t MMOT_OPEN_LOAD(void); 

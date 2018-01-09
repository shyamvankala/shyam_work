 #include "stdint.h"
#include "std_init.h"
#include "ZMDCM0.h"
#include "SPI.h"
#include "ADC.h"
#include "ADC_cfg.h"

#define LIN_CHANGED_FLAG_DATA_SIZE      35

// #define  _LEFT_MIRROR
 extern l_u8    l_pChangedFlagData[LIN_CHANGED_FLAG_DATA_SIZE];

 #define HTR_ICM_AMPS_TO_LSBS     ICM_AMPS_TO_LSBS_HS1
//#define ICM_AMPS_TO_LSBS_OUT1(val)    ((const uint16)(((val)*1000.)/I_CM_MA_PER_LSB_OUT1))
//#define ICM_AMPS_TO_LSBS_OUT2(val)    ((const uint16)(((val)*1000.)/I_CM_MA_PER_LSB_OUT2))
//#define ICM_AMPS_TO_LSBS_OUT3(val)    ((const uint16)(((val)*1000.)/I_CM_MA_PER_LSB_OUT3))
//#define ICM_AMPS_TO_LSBS_OUT4(val)    ((const uint16)(((val)*1000.)/I_CM_MA_PER_LSB_OUT4))
//#define ICM_AMPS_TO_LSBS_OUT5(val)    ((const uint16)(((val)*1000.)/I_CM_MA_PER_LSB_OUT5))
//#define ICM_AMPS_TO_LSBS_OUT6(val)    ((const uint16)(((val)*1000.)/I_CM_MA_PER_LSB_OUT6))
//#define ICM_AMPS_TO_LSBS_OUT7(val)    ((const uint16)(((val)*1000.)/I_CM_MA_PER_LSB_OUT7))
//#define ICM_AMPS_TO_LSBS_OUT8_L(val)  ((const uint16)(((val)*1000.)/I_CM_MA_PER_LSB_OUT8_L))
//#define ICM_AMPS_TO_LSBS_OUT8_H(val)  ((const uint16)(((val)*1000.)/I_CM_MA_PER_LSB_OUT8_H))
#define ICM_AMPS_TO_LSBS_HS1(val)       ((const uint16)((val)*1000))
 # define MPOT_R_NOM                3700.  // Nominal potentiometer value
# ifdef _LEFT_MIRROR
#  define MPOTX_INVERT             0
#  define MPOTY_INVERT             1

//#  define MMOT_LEFT                eNdxAIN_HB4H
#  define MMOT_LEFT                eNdxAIN_HB2L
//#  define MMOT_RIGHT               eNdxAIN_HB4L
#  define MMOT_RIGHT                eNdxAIN_HB2H
//#  define MMOT_UP                  eNdxAIN_HB3H
#  define MMOT_UP	                eNdxAIN_HB2L
//#  define MMOT_DOWN                eNdxAIN_HB3L
#  define MMOT_DOWN                eNdxAIN_HB2H

# elif defined(_RIGHT_MIRROR)
#  define MPOTX_INVERT             1
#  define MPOTY_INVERT             1
//#  define MMOT_LEFT                eNdxAIN_HB4H
#  define MMOT_LEFT                eNdxAIN_HB2L
//#  define MMOT_RIGHT               eNdxAIN_HB4L
#  define MMOT_RIGHT                eNdxAIN_HB2H
//#  define MMOT_UP                  eNdxAIN_HB3H
#  define MMOT_UP	                eNdxAIN_HB2L
//#  define MMOT_DOWN                eNdxAIN_HB3L
#  define MMOT_DOWN                eNdxAIN_HB2H
# endif

 #if defined(_LEFT_MIRROR)
# define ADC_CH_MPOTX          eNdxAin2RAT
# define ADC_CH_MPOTY          eNdxAin1RAT
#elif defined(_RIGHT_MIRROR)
# define ADC_CH_MPOTX          eNdxAin1RAT
# define ADC_CH_MPOTY          eNdxAin2RAT
#else
# error _XXX_MIRROR not defined
#endif

/*
const uint8 LIN_CTRL_SELECT_ASP_XXXX = 
#ifdef _LEFT_MIRROR // left mirror
  LIN_CTRL_SELECT_ASP_LEFT;
#endif
#ifdef _RIGHT_MIRROR // right mirror
  LIN_CTRL_SELECT_ASP_RIGHT;
#endif */

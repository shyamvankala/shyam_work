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
 *  @file      hc2.c
 *
 *  @desc      Sourcefile for HC2 (Heading control) LED functions (control and diagnostics)
 *
 *  @document  Integrated_LINBus_Software_Design.docx
 *
 * =============================================================================
 *  Revision History
 *  ===============
 *   Date    Initials   Version   	Change
 * 24.07.2017	   SH		 0.0.1    	Initial version
 
 * =============================================================================
*/
#include "stdint.h"
#include "ZMDCM0.h"
#include "SPI.h"
#include "PWM.h"
#include "SBC.h"
#include "SBC_cfg.h"
#include "HS_cfg.h"
#include "HS.h"
#include "LINCom.h"
#include "SBC_cfg.h"
#include "LINCom_cfg.h"
#include "Applinit.h"
#include "Vmon.h"
#include "TMR32_Cfg.h"
#include "TMR32.h"
#include "params.h"
#include "Timer.h"
#include "Hc2.h"
#include "LinAppl.h"
#include "Error.h"
 // For voltage compensation calculation
#define INTERNAL_FACTOR 	0x400 // Possible with shift-operation
#define	 DENOMINATOR_FACT 	(const uint32_t)(((double)HC2_LED_CURR_100_PERC_MA*HC2_PWM_100_PERCENT_NOM*HC2_LEDCURV_R_DYN*INTERNAL_FACTOR)/(LIN_CTRL_HC2_DIMMING_MAX*UBAT_MEAS_MV_PER_LSB))
#define	 NOMINATOR_ADD 		(const int16_t)((HC2_LEDCURV_I0_MA*HC2_LEDCURV_R_DYN-HC2_LEDCURV_U0_MV)/UBAT_MEAS_MV_PER_LSB)

// assignment of suberror-index
#define HC2_ERRIDX_HC2_SUP_OC         0 // Overcurrent on highside switch (OC flag of L99MM70 set when HS is ON)
#define HC2_ERRIDX_HC2_SUP_TO_UBAT    1 // HC2_SUP shorted to Ubat (detected when HS=OFF and LS=OFF and ADC is > Ubat-Uledmin)
#define HC2_ERRIDX_HC2_OPEN_LOAD      2 // HC2_SUP or HC2_GND open wire (detected when HS=ON and LS=OFF and no overcurrent and ADC ~0V)
#define HC2_ERRIDX_HC2_GND_TO_GND     3 // HC2_GND shorted to GND (detected when HS=ON and LS=OFF and not L99MM70 UC Flag and ADC ~0V)
#define HC2_ERRIDX_HC2_GND_TO_UBAT1   4 // HC2_GND shorted to Ubat (detected when HS=OFF and LS=OFF and ADC ~Ubat)
#define HC2_ERRIDX_HC2_GND_TO_UBAT2   5 // HC2_GND shorted to Ubat    (detected when HS=ON and LS=OFF and     HC2_ERRIDX_HC2_GND_TO_UBAT1)
#define HC2_ERRIDX_HC2_SUP_TO_HC2_GND 6 // HC2_SUP shorted to HC2_GND (detected when HS=ON and LS=OFF and not HC2_ERRIDX_HC2_GND_TO_UBAT1)



# define HC2_NUM_SUB_ERRORS            7 //HC2sup_to_HC2_GND diagnosis error is enable

// Substates
#define HC2_SUBSTATE_PRE       0 // Preflash is active
#define HC2_SUBSTATE_PREPAUSE  1 // Pause between preflash and mainflash
#define HC2_SUBSTATE_MAIN      2 // Mainflash is active
#define HC2_SUBSTATE_MAINPAUSE 3 // Pause between mainflash and next preflash
#define HC2_SUBSTATE_IDLE      4 // idle phase with constant brightness

typeHc2State Hc2State;

static uint32_t LedBrightnessNew,LedBrightness;
static uint16_t LedBrightnessReq, LedBrightnessPrev, Ramp_Slope;
static uint8_t  Ramp_Calc;
static typeTimerValue Hc2Timer;
static uint8_t ParamIdx,ParamIdx1; // for moduleglobal usage

static const struct{
  uint8_t tMain;  // P1: duration mainflash
  uint8_t tPre ;  // P2: duration preflash
  uint8_t tPause; // P3: duration pause between two pulsegroups
  uint8_t fPre ;  // P5: Factor for preflash (0.01..1.00) (Ipre = base*P4*P5)
  uint8_t fMain;  // P4: Factor for mainflash 1..26.5 = 0.255
  uint8_t nReps;  // P6: number of repetitions for pulsegroups 
  uint8_t tIdle;  // P7: Duratin of idle phase with basic intensity
}Hc2Params[3] = {
// P1                                | P2                                                 | P3                              | P5                    | P4                       | P6               | P7
  {HC2_ST1_P1_T_MAIN_MS/TIMER_CYLE_MS, HC2_ST1_P1_T_MAIN_MS/TIMER_CYLE_MS/HC2_ST1_P2_Q_PRE, HC2_ST1_P3_T_PAUSE/TIMER_CYLE_MS, HC2_ST1_P5_F_PREF*100., HC2_ST1_P4_F_MAIN*10.-10., HC2_ST1_P6_N_REPS, HC2_ST1_P7_T_IDLE_MS/TIMER_CYLE_MS},
  {HC2_ST2_P1_T_MAIN_MS/TIMER_CYLE_MS, HC2_ST2_P1_T_MAIN_MS/TIMER_CYLE_MS/HC2_ST2_P2_Q_PRE, HC2_ST2_P3_T_PAUSE/TIMER_CYLE_MS, HC2_ST2_P5_F_PREF*100., HC2_ST2_P4_F_MAIN*10.-10., HC2_ST2_P6_N_REPS, HC2_ST2_P7_T_IDLE_MS/TIMER_CYLE_MS},
  {HC2_ST3_P1_T_MAIN_MS/TIMER_CYLE_MS, HC2_ST3_P1_T_MAIN_MS/TIMER_CYLE_MS/HC2_ST3_P2_Q_PRE, HC2_ST3_P3_T_PAUSE/TIMER_CYLE_MS, HC2_ST3_P5_F_PREF*100., HC2_ST3_P4_F_MAIN*10.-10., HC2_ST3_P6_N_REPS, HC2_ST3_P7_T_IDLE_MS/TIMER_CYLE_MS}
};

static typeErrorState        aHc2SubErrorState[HC2_NUM_SUB_ERRORS]; // Array with sub-error state variables
static typeDebounceState     aHc2DebounceState[HC2_NUM_SUB_ERRORS]; // State variables for debouncing
static typeDebounceCnt       aHc2DebounceCnt[HC2_NUM_SUB_ERRORS];   // Counter variables for debouncing
static const typeDebounceCfg aHc2DebounceCfg[HC2_NUM_SUB_ERRORS] = { // Configuration for debouncing

// State                                          |Timer/Counter                                   | Set delay ticks                      | Reset delay ticks                    | Action                   | Action parameter
{&aHc2DebounceState[HC2_ERRIDX_HC2_SUP_OC]        , &aHc2DebounceCnt[HC2_ERRIDX_HC2_SUP_OC]        , HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, DEBOUNCE_ACTION_SUB_ERROR, &aHc2SubErrorState[HC2_ERRIDX_HC2_SUP_OC]},
{&aHc2DebounceState[HC2_ERRIDX_HC2_SUP_TO_UBAT]   , &aHc2DebounceCnt[HC2_ERRIDX_HC2_SUP_TO_UBAT]   , HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, DEBOUNCE_ACTION_SUB_ERROR, &aHc2SubErrorState[HC2_ERRIDX_HC2_SUP_TO_UBAT]},
{&aHc2DebounceState[HC2_ERRIDX_HC2_OPEN_LOAD]     , &aHc2DebounceCnt[HC2_ERRIDX_HC2_OPEN_LOAD]     , HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, DEBOUNCE_ACTION_SUB_ERROR, &aHc2SubErrorState[HC2_ERRIDX_HC2_OPEN_LOAD]},
{&aHc2DebounceState[HC2_ERRIDX_HC2_GND_TO_GND]    , &aHc2DebounceCnt[HC2_ERRIDX_HC2_GND_TO_GND]    , HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, DEBOUNCE_ACTION_SUB_ERROR, &aHc2SubErrorState[HC2_ERRIDX_HC2_GND_TO_GND]},
{&aHc2DebounceState[HC2_ERRIDX_HC2_GND_TO_UBAT1]  , &aHc2DebounceCnt[HC2_ERRIDX_HC2_GND_TO_UBAT1]  , HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, DEBOUNCE_ACTION_SUB_ERROR, &aHc2SubErrorState[HC2_ERRIDX_HC2_GND_TO_UBAT1]},
{&aHc2DebounceState[HC2_ERRIDX_HC2_GND_TO_UBAT2]  , &aHc2DebounceCnt[HC2_ERRIDX_HC2_GND_TO_UBAT2]  , HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, DEBOUNCE_ACTION_SUB_ERROR, &aHc2SubErrorState[HC2_ERRIDX_HC2_GND_TO_UBAT2]},
{&aHc2DebounceState[HC2_ERRIDX_HC2_SUP_TO_HC2_GND], &aHc2DebounceCnt[HC2_ERRIDX_HC2_SUP_TO_HC2_GND], HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, HC2_ADC_DEBOUNCE_MS/HC2_CYCLE_TIME_MS, DEBOUNCE_ACTION_SUB_ERROR, &aHc2SubErrorState[HC2_ERRIDX_HC2_SUP_TO_HC2_GND]}
};


void Ramp(uint16_t Target, uint16_t slope);
 uint8_t HC2_UBAT,HC2_Ubat_open_load ;
static uint32_t CalcBrightnessPreFlash( void);
static uint32_t CalcBrightnessMainFlash( void);
//TODO : SHYAM HAS TO PROVIDE COMMENTS AND REMOVE WARNINGS
 static enum {
  DIAG_STIMUL_OFF,
  DIAG_STIMUL_ON
} DiagStimulState;
static uint16 DiagStimulCount;
/**
  * @brief  This function Initializes all variables to reset state.
  * @param  None
  * @retval None
*/
 
void Hc2Init( void){

  Hc2State.s.state = HC2_STATE_OFF;
  Ramp_Calc = 0;
  HC2_UBAT=OFF;
 HC2_Ubat_open_load=OFF;
   DebounceInit( aHc2DebounceCfg, HC2_NUM_SUB_ERRORS);
 //  ErrorArrayInit( aHc2SubErrorState, HC2_NUM_SUB_ERRORS);	  //need to check the error bit 

   DiagStimulCount = HC2_DIAGSTIMUL_FIRST_MS/HC2_CYCLE_TIME_MS;
   DiagStimulState = DIAG_STIMUL_OFF;
}
/**
  * @brief  This function must be called cyclically every HC2_CYCLE_TIME_MS.
  *         Updates application (checks inputs and updates statemachine and outputs) and diagnostic functions for HC2 LED.
  * @param  None
  * @retval None
*/

 extern  volatile eHS_ShortCircuitType eLoadStatus;
 extern volatile uint8_t u8ErrorCnt;
 volatile	static	 uint16_t hc2_curr_raw , updated_Hc2_curr_raw ;
void Hc2Cyclic( void){

    boolean LedOnNew;
	uint8_t Ramp_Reset = 0; 
  static uint8_t  Hc2_cntr;
#if HC2_BRIGHTNESS==LINEAR        // for linear brightness
    if( l_u8_rd_CTR_DIM_DSE_LNCH_LIN() <= 50){
      LedBrightnessReq = l_u8_rd_CTR_DIM_DSE_LNCH_LIN();
    }
    else if( l_u8_rd_CTR_DIM_DSE_LNCH_LIN() <= 230){
      LedBrightnessReq = 50+(l_u8_rd_CTR_DIM_DSE_LNCH_LIN()-50)*7;
    } 
    else {
      LedBrightnessReq = 1310+(l_u8_rd_CTR_DIM_DSE_LNCH_LIN()-230)*8;
    }
		LedBrightness= l_u8_rd_CTR_DIM_DSE_LNCH_LIN();
#endif

#if HC2_BRIGHTNESS==QUADRATIC     // for quadratic brightness
    {
      uint16 Bright_temp;
      Bright_temp = l_u8_rd_CTR_DIM_DSE_LNCH_LIN()*l_u8_rd_CTR_DIM_DSE_LNCH_LIN();
      LedBrightnessReq = Bright_temp;
    }
#endif

	if( Hc2State.s.state != HC2_STATE_ON) {
    LedOnNew = FALSE; // UK: switched off only if the previous state was Off
  }


	if( VTFailure() ||                                                  // Voltage or temperaturefailure
    (l_u8_rd_BRC_CTR_EXMI_LIN() == LIN_CTRL_SELECT_ASP_SNV) ||
    (l_u8_rd_CTR_DSE_1_XH_WARN_LNCH_LIN() < LIN_CTRL_HC2_MODE_ON) ||  // Mode is off
    (l_u8_rd_CTR_DSE_1_XH_WARN_LNCH_LIN() > LIN_CTRL_HC2_MODE_BL3)|| // Mode is invalid or SNV
    (l_u8_rd_CTR_DIM_DSE_LNCH_LIN() == LIN_CTRL_HC2_DIMMING_SNV)){    // dimming signal is not available
		    if( ( Hc2State.s.state == HC2_STATE_ON) && ( LedBrightnessNew > 0)) { // New change request P8 and P9
		      LedOnNew = TRUE;                                                    // HC2 On state => Off State over Ramp
		      if(Ramp_Calc == 0) {
		        Ramp_Slope = (uint16)(((uint32)(LedBrightnessNew*MAIN_CYCLE_MS)/HC2_P9_RAMP_DOWN_MS)+1);
		        Ramp_Calc = 1;
		      }
		      Ramp(Ramp_Reset, Ramp_Slope);                                // If the Brighness is > 0 then slope down
		      LedBrightnessPrev = (uint16) LedBrightnessNew;
		    }                                                                     // untill the Brighness = 0
		    else {
		      Hc2State.s.state = HC2_STATE_OFF;
		      l_u8_wr_ST_DSE_1_XH_WARN_LNCH_LIN( LIN_HC2_FEEDBACK_OFF); // May be overwritten later by diagnostics result
		      Ramp_Calc = 0;
		    }
		  }
 

	  /* daignostic part skiped */ 


	else if( l_u8_rd_CTR_DSE_1_XH_WARN_LNCH_LIN() == LIN_CTRL_HC2_MODE_ON){ // constant brightness requested
	    
		    if(Hc2State.s.state == HC2_STATE_FLASH) { // state transition from blinking -> constant brightness
		      LedBrightnessNew = LedBrightnessReq;
		    } 
		    else {
		      if (LedBrightnessReq < 1){
		        LedBrightnessReq = 1;
		      } 
		
		      if(LedBrightnessPrev < LedBrightnessReq) {
		        Ramp_Slope = (uint16)((((uint32)(LedBrightnessReq - LedBrightnessPrev)*MAIN_CYCLE_MS)/HC2_P8_RAMP_UP_MS)+1);
		      } 
		      else if (LedBrightnessPrev > LedBrightnessReq) {
		        Ramp_Slope = (uint16)((((uint32)(LedBrightnessPrev - LedBrightnessReq)*MAIN_CYCLE_MS)/HC2_P9_RAMP_DOWN_MS)+1);
		      }
		      Ramp(LedBrightnessReq, Ramp_Slope);
		    }
		    Hc2State.s.state = HC2_STATE_ON;
		    LedOnNew = TRUE;
		    LedBrightnessPrev = LedBrightnessReq; 
		    l_u8_wr_ST_DSE_1_XH_WARN_LNCH_LIN( LIN_HC2_FEEDBACK_ON); // May be overwritten later by diagnostics result
			}


	  else{ // l_u8_rd_CTR_DSE_1_XH_WARN_LNCH_LIN() == LIN_CTRL_HC2_MODE_BL1..3
		    ParamIdx = l_u8_rd_CTR_DSE_1_XH_WARN_LNCH_LIN() - LIN_CTRL_HC2_MODE_BL1; // Calculate index for parametertable
		    l_u8_wr_ST_DSE_1_XH_WARN_LNCH_LIN( l_u8_rd_CTR_DSE_1_XH_WARN_LNCH_LIN()); // May be overwritten later by diagnostics result
		    if( Hc2State.s.state != HC2_STATE_FLASH){ // blinking requested new -> initialize states
		      Hc2State.s.state = HC2_STATE_FLASH;
		      Hc2State.s.sub = HC2_SUBSTATE_PRE;
		      Hc2State.s.cnt = 0;
		      Hc2Timer = TimerGet();
		    }
			// Update statemachine
		    switch( Hc2State.s.sub){
		    case HC2_SUBSTATE_PRE:
		      if( TimerGetDiffTicks( Hc2Timer) >= Hc2Params[ParamIdx].tPre){
		        // Next phase
		        Hc2State.s.sub = HC2_SUBSTATE_PREPAUSE;
		        Hc2Timer = TimerGet();
		      }
		      else{
		        LedOnNew = TRUE;
		        LedBrightnessNew = CalcBrightnessPreFlash();
		      }
		      break;
		    case HC2_SUBSTATE_PREPAUSE:
		      if( TimerGetDiffTicks( Hc2Timer) >= Hc2Params[ParamIdx].tPre){
		        // Start with mainflash
		        LedOnNew = TRUE;
		        LedBrightnessNew = CalcBrightnessMainFlash();
		        Hc2State.s.sub = HC2_SUBSTATE_MAIN;
		        Hc2Timer = TimerGet();
		      }
		      break;
		    case HC2_SUBSTATE_MAIN:
		      if( TimerGetDiffTicks( Hc2Timer) >= Hc2Params[ParamIdx].tMain){
		        // Start pause between pulse-groups
		        Hc2State.s.sub = HC2_SUBSTATE_MAINPAUSE;
		        Hc2Timer = TimerGet();
		      }
		      else{
		        LedOnNew = TRUE;
		        LedBrightnessNew = CalcBrightnessMainFlash();
		      }
		      break;
		    case HC2_SUBSTATE_MAINPAUSE:
		      if( TimerGetDiffTicks( Hc2Timer) >= Hc2Params[ParamIdx].tPause){
		        if( Hc2State.s.cnt >= Hc2Params[ParamIdx].nReps){
		          Hc2State.s.sub = HC2_SUBSTATE_IDLE;
		          LedBrightnessNew = LedBrightnessReq;
		        }
		        else{
		          Hc2State.s.cnt++;
		          Hc2State.s.sub = HC2_SUBSTATE_PRE;
		          LedBrightnessNew = CalcBrightnessPreFlash();
		        }
		        LedOnNew = TRUE;
		        Hc2Timer = TimerGet();
		      }
		      break;
		    case HC2_SUBSTATE_IDLE:
		      if( TimerGetDiffTicks( Hc2Timer) >= Hc2Params[ParamIdx].tIdle){
		        Hc2State.s.sub = HC2_SUBSTATE_PRE;
		        LedBrightnessNew = CalcBrightnessPreFlash();
		        Hc2Timer = TimerGet();
		        Hc2State.s.cnt = 0;
		      }
		      else{
		        LedBrightnessNew = LedBrightnessReq;
		      }
		      LedOnNew = TRUE;
		      break;
		    }
		    if( LedBrightnessNew > LIN_CTRL_HC2_DIMMING_MAX){ // limit to max.
		      LedBrightnessNew = LIN_CTRL_HC2_DIMMING_MAX;
		    } 
  		}
		
	
		 /* daignostic part  */ 
	 if( VTFailure()){ // no diagnostics, reset debounce counters
    DebounceInit( aHc2DebounceCfg, HC2_NUM_SUB_ERRORS);
    l_u8_wr_ST_DSE_1_XH_WARN_LNCH_LIN( LIN_HC2_FEEDBACK_OFF);
 		 }		 
	 else {
	 	if (LedOnNew==ON) {	     //diagnostic check in ON mode 
			  if( ApplReadCurrentMonitorRaw(HC2_CM, &hc2_curr_raw) == E_OK){
			
				if (Hc2_cntr<100 )				  //Read the current of the HC2 on an sampling of for 1Sec 
				{
				 	updated_Hc2_curr_raw += hc2_curr_raw;
				 }
				 else 
				 {
						 if 	(updated_Hc2_curr_raw <150)
						{
							HC2_Ubat_open_load =ON;
						}
						else {
								HC2_Ubat_open_load=OFF;}
								updated_Hc2_curr_raw =0;
								Hc2_cntr=0;			
						}				
				Hc2_cntr++;	
		 }
	  }

	 if (	HC2_UBAT==OFF)	  //diagnostic check in Hc2 Off mode 
		{

			  	eLoadStatus = HS_eGetHSError(eHS2);
			switch (eLoadStatus)
			{
				case eHS_Normal:// No error
				{
				u8ErrorCnt=0;
				HC2_Ubat_open_load =OFF;
				}
				break;
	
				case eHS_SCG:// Short to Ground, could be detected only in Driver is ON
				{
					u8ErrorCnt+=1;
				}
				break;
		
				case eHS_OpenLoad:// Load missing - could be detected if Driver is OFF
				{
					u8ErrorCnt +=2;
					HC2_Ubat_open_load =ON;
	
				}
				break;
		
				case eHS_ShortVDDE:// Short to VDDE - could be detected if Driver is OFF
				{
					u8ErrorCnt+=3;
				}
				break;
			}	  
		}

		
			
		//update the error bit in the register of Open oad Err_05 	

				 DebounceUpdate(
          &(aHc2DebounceCfg[HC2_ERRIDX_HC2_OPEN_LOAD]), // Pointer to configuration
          FALSE,                     // Condition for immediate detection of failure (not used)
          (eLoadStatus==eHS_OpenLoad)&&(HC2_Ubat_open_load ==ON),
          FALSE,                     // Condition for immediate reset of failure (not used)
         (eLoadStatus==eHS_Normal)||(HC2_Ubat_open_load ==OFF)); // Condition for delayed reset of failure
          
	
	  if( ErrorArrayUpdateResult( aHc2SubErrorState, HC2_NUM_SUB_ERRORS, LIN_ERR_IDX_HC2) == LIN_ERR_ERR){ // HC2-Error detected
      l_u8_wr_ST_DSE_1_XH_WARN_LNCH_LIN( LIN_HC2_FEEDBACK_ERR); // send feedback error
   	 }
   
	 // Stimulation for diagnostics
    if(((Hc2State.s.state == HC2_STATE_OFF) || ((Hc2State.s.state == HC2_STATE_FLASH) && (Hc2State.s.sub == HC2_SUBSTATE_MAINPAUSE))) && 
         (aHc2SubErrorState[HC2_ERRIDX_HC2_GND_TO_GND] != ERROR_STATE_ERROR)){ // Stimulate only if no short of HC2_GND to GND is detected (otherwise the driver may be confused by the flashing LED)
       if( --DiagStimulCount == 0){
         if( DiagStimulState == DIAG_STIMUL_OFF){
           DiagStimulState = DIAG_STIMUL_ON;
           DiagStimulCount = HC2_DIAGSTIMUL_TIME_MS/HC2_CYCLE_TIME_MS;
         }
         else{
           DiagStimulState = DIAG_STIMUL_OFF;
           DiagStimulCount = HC2_DIAGSTIMUL_OTHER_MS/HC2_CYCLE_TIME_MS;
         }
       }
       if( DiagStimulState == DIAG_STIMUL_ON){
         LedOnNew = TRUE;
         LedBrightnessNew = 0;
       }
    }
  }
	 
	   // Update duty cycle and period for 1khz 
  if( LedOnNew){
		    uint32 temp;
		    uint16 period, duty_percent;
			uint16 duty=0;
		
		    if( LedBrightnessNew == 0){
			  PWM_vSettings(HC2_PWM_100_PERCENT_NOM,0);
		   
		    }
		    else{
		      temp = (LedBrightnessNew*(DENOMINATOR_FACT))/(VmonGetUbatRaw()+NOMINATOR_ADD);
		    //  duty = (uint16)((temp/INTERNAL_FACTOR)+15);
				 duty = LedBrightness *0.393;
		      period = (uint16)((duty*(uint32)HC2_PWM_100_PERCENT_NOM*INTERNAL_FACTOR)/temp);
			  duty_percent=(uint16) ((duty*100)/period)	;
		    }  
			
			// final check if in diagnostic mode any error then sawitch it off else switch on 
			//  if (HC2_Ubat_open_load ==OFF)
			  	{
					  HC2_UBAT_OUT(1);
					  HC2_UBAT=ON ;
					  PWM_vSettings(HC2_PWM_100_PERCENT_NOM, duty);
			 	 }
			 
		  }

		  else{
		    HC2_UBAT_OUT(0);
			HC2_UBAT=OFF;
		   	PWM_vSettings(HC2_PWM_100_PERCENT_NOM,0);
		  }			

}
/**
  * @brief  This function Calculate brightness for preflash.Formula: Brightness preflash = Brightness_mainflash * (P5 * 0.01)
  * @param  None
  * @retval uint32: brightness level.
*/
static uint32_t CalcBrightnessPreFlash( void){
  return (((LedBrightnessReq*(uint32)(Hc2Params[ParamIdx].fMain+10))*Hc2Params[ParamIdx].fPre)/1000);
}

/**
  * @brief  This function Calculate brightness for mainflash.Formula: Brightness mainflash = LIN_Dimvalue * (P4 + 10)/10
  * @param  None
  * @retval uint32: brightness level.
*/
static uint32_t CalcBrightnessMainFlash( void){
  return ((LedBrightnessReq*(uint32)(Hc2Params[ParamIdx].fMain+10))/10);
}
/**
  * @brief  This function makes ramp increases or decreases a value with given slope depending on 
            the actual State with respect to the target.
  * @param  None//TODO : To be filled by Shyam
  * @retval uint32: brightness level.
*/
void Ramp(uint16 Target, uint16 Slope)
{
	if (LedBrightnessNew < Target)
	{
	    if ( Target < LedBrightnessNew + Slope) // To avoid the overlimit
	    LedBrightnessNew = Target;
	    else
		  LedBrightnessNew += Slope;
	}
	else if (LedBrightnessNew > Target)              // To avoid the underlimit
	{
	    if ( LedBrightnessNew < Target + Slope)
	    LedBrightnessNew = Target;
	    else
		  LedBrightnessNew -= Slope;
	}
}

/*-- End of File --*/

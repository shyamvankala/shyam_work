 /**********************************************************************************************************************************
Project     :  L7-Mirror
Filename    :  Params.h
Brief descr.:  Definition of precompile parameters.
Copyright   :  
Remarks     :  Some parameters are provided by command line option to compiler. (-Dxxx) 
               e.g.: -D_LEFT_MIRROR -D_DEBUG=1 -D_EMC_TEST=0 -D_FBLSUP=1 -D_FBLCODE=1
                     -DPFLD_AVAILABLE=1 -DEC_AVAILABLE=1 -DHC2_AVAILABLE=1
Version
  1.0.0  18.09.2009  UF  Creation
  1.0.1  14.01.2010  UF  Changed some comments for HC2 parameters
  1.0.2  18.01.2010  UF  Added halfbridge extended diagnostics
  1.0.3  08.11.2010  UK  Powerfold, HC2 modifications
  1.0.4  12.11.2010  UF  Added PFLD_CURR_RUN_MIN_AMPS and PFLD_CURR_SAMPLING_US
  1.0.5  17.11.2010  UK  Version information on LIN, powerfold block current adjustment
  1.0.6  02.12.2010  UF  Moved mirror configuration variant to compiler-options
  1.0.7  08.03.2011  UK  Parameters added for reactivation of powerfold
**********************************************************************************************************************************/

#ifndef _PARAMS_H_
#define _PARAMS_H_

/* Include-Files ################################################################################################################*/
/* Defines ######################################################################################################################*/
// Parameter

// Software Version over LIN Protocol - Error Message
#define SOFTWARE_VERSION_LIN 2

// LIN Version 1 (for 1.3) 
// LIN Version 0 (for 2.0/2.1)
#define LIN_VERSION 0

// Internal parameters
#define TIMER_CYLE_US 10000  // One timer tick lasts this value in microseconds
#define MAIN_CYCLE_MS 10 // Cycletime for main-loop in ms
#define HARDWARE_VERSION_MAIN	2 	// Sent on LIN / Sensor Identification
#define HARDWARE_VERSION_SUB	0x75  // Sent on LIN / Sensor Identification
#define F_BUS_KHZ 16000 // If a different value shall be used, the initialization of the PLL in hardware.c must be changed.
#define SPI_CLK_FREQ_KHZ 1000 // 1MHz SPI-Clock for L99MM70
//#define SPI_CLK_FREQ_KHZ 250 // 250 kHz SPI-Clock for L99MM70
#define L99_CURRENT_MONITOR_SWITCH_DELAY_US 1000 // minimum time for CM-voltage to settle after a mux-switch 
#define HBDIAG_EXTENDED OFF // Extended diagnostics for halfbridges

// Select the hardware type (see BootLdrTypes.h)
#define _HARDWARE_TYPE HW_TYPE_G2X_LIN

// Select the brightness based on linear or quadratic variation
// options: LINEAR or QUADRATIC
#define LINEAR 0
#define QUADRATIC 1
#define HC2_BRIGHTNESS LINEAR

// Definition of mirror glass unit (see BootLdrTypes.h)
#define MM_TYPE      	MM_TYPE_MM5_1456301
#define BMW_NUMBER		BMW_NR_7480653
#define BMW_NUMBER_BCD	{0x00,0x07,0x48,0x06,0x53}

// LIN-Timeout 
#define TIMEOUT_BRC_CTR_ASP_LIN_MS 5000 // Timeout for LIN-Frame BRC_CTR_ASP_LIN. After that time all Signals are set to SNV 

// Parameters for Mirror-Glass movement
#define MMOT_CURR_MAX_AMPS     0.500   // maximum current
#define MMOT_CURR_MAX_TIME_MS    300   // detection time for maximum current measured
#define MMOT_TOTAL_TIME_MAX_MS 10000   // Timeout for total time
#define MMOT_STALL_DET_INIT_MS 2000   // Timeout for change of potentiometer (initial)
#define MMOT_STALL_DET_RUN_MS    900   // Timeout for change of potentiometer (running)
#define MMOT_BRAKE_TIME_MS        60   // Time for braking (both terminals to GND or UBAT) after running engine
#define MPOT_ERROR_TH_PERCENT      3.  // Potentiometer error detection: invalid range to limit MPOT_SUP or MPOT_GND (% of nominal)
#define MPOT_ERROR_SUP_MIN        87.  // Potentiometer error detection: invalid range from MPOT_SUP to VCC (% of nominal)
#define MPOT_ERROR_SUP_MAX        96.5 // Potentiometer error detection: invalid range from MPOT_SUP to VCC (% of nominal)
#define MPOT_ERROR_DEBOUNCE_MS   500   // Debounce time for potentiometer error detection (invalid high and low range) 
#define MPOT_BRAKE_BEFORE_LSBS     10    //Modified for ZAMC4200 //1   // Brake xxx LSBs before target position is reached
#define MPOT_MIN_DEVIATION_LSBS    1   // Minimum deviation for starting the motor (if deviation less than or equal to this, the motor for this axis is not switched on)
#define MPOT_MIN_CHANGE_RUN_LSBS   2   // Minimum change of poti in LSBs for motor move detection. (1 is unsafe due to possible toggling of 1 LSB without moving)

// Parameters for Powerfold
#define PFLD_CURR_MAX_AMPS     4.500 //3.000  // maximum current
#define PFLD_CURR_MAX_TIME_MS    100   // detection time for maximum current measured

// UK change
#define PFLD_CURR_RUN_MIN_AMPS    0.400   // minimum rush-in current
#define PFLD_CURR_SAMPLING_US     1000    // Sampling period during for rush-in current
#define PFLD_PEAK_TIME_MS         40      // detection for Startup current in the beginning
#define PFLD_BRAKE_TIME_MS        40      // Time for braking (both terminals to GND or UBAT) after block detection for deenergizing of coil
#define PFLD_REACTIVATE_TRIGGER   12.     // Minimum voltage to re-activate the powerfold motor if the prerequisite is met
#define PFLD_REACTIVATE_SET       10.     // Voltage range under which the re-activation of powerfold motor is required


// Internal parameters
#define TIMER_CYLE_US 10000  // One timer tick lasts this value in microseconds
//#define MAIN_CYCLE_MS 5 // Cycletime for main-loop in ms



#define PFLD_CURR_RUN_MIN_AMPS    0.400   // minimum rush-in current


// Parameters for Heater
//#define HTR_CURR_MAX_AMPS      2.800   // maximum current
#define HTR_CURR_MIN_AMPS       650    // minimum current
#define HTR_CURR_MAX_TIME_MS    1000   // detection time for maximum current measured (or MCU-Flag immediately), integrated over several PWM-Periods
#define HTR_CURR_MIN_TIME_MS    300   // detection time for open load flag
#define HTR_NOMINAL_POWER_W       18.  // Nominal power in watts for 100% mirror heater (18 or 30)
#define HTR_PWM_PERIOD_MS       600
#define HTR_OPENLoad_TIME_MS    1100
#define HTR_OC_debounce_TIME_MS 300
// Parameters for EC-Glass
#define EC_CURR_MAX_AMPS       0.450   // maximum current
#define EC_CURR_MAX_TIME_MS      300   // detection time for maximum current measured
#define EC_OVERCURR_DELAY_MS   10000   // Delay after overcurrent before reactivation
#define EC_VMON_DET_DELAY_MS    1000   // Delay for ECV-Status (voltage monitoring ECV-Pin L99MM70) until error detection
#define EC_VMON_HIGH_PERCENT_MIN  50   // Minimum value of dimming in percent to evaluate L99MM70 "ECV Voltage High"
#define EC_ADC_DEBOUNCE_MS       200   // Debounce time for ADC monitoring of ECV voltage
#define EC_CLEARING_TIME_MS    10000   // Time for activation of lowside switch after start of clearing (0 for infinite activation)
#define EC_FLOATING_TIME_MS     1000   // Time for deactivation of lowside switch after clearing is finished (0 for infinite activation)

// Parameters for HC2
#define HC2_CURR_MAX_AMPS      0.250   // maximum current
#define HC2_CURR_MAX_TIME_MS     300   // detection time for maximum current measured
#define HC2_PWM_100_PERCENT_NOM  1000  //2000   

#define HC2_LEDCURV_R_DYN         122.  // Led-curve data: dynamic resistance

// L7 HC2
#ifdef _LEFT_MIRROR
#define HC2_LEDCURV_U0_MV         10000.  // Led-curve data: U0 voltage at I0 in millivolts
#define HC2_LEDCURV_I0_MA         28.8    // Led-curve data: I0 current at U0 in milliamperes
#define HC2_LED_CURR_100_PERC_MA  30.     // LED current to be expected for 100% => 1500 CD/m²
#endif
#ifdef _RIGHT_MIRROR
#define HC2_LEDCURV_U0_MV         10000.  // Led-curve data: U0 voltage at I0 in millivolts
#define HC2_LEDCURV_I0_MA         28.8    // Led-curve data: I0 current at U0 in milliamperes
#define HC2_LED_CURR_100_PERC_MA  30.     // LED current to be expected for 100% => 1500 CD/m²
#endif

//#define HC2_DIAG_OC_MIN_RESET_US 100   // Minimum on time for lowside switch to reset overcurrent error
#define HC2_DIAG_ULED_MAX_SET      6.4 // Maximum voltage for LEDs when diag-current flows (set)
#define HC2_DIAG_ULED_MAX_RESET    6.6 // Maximum voltage for LEDs when diag-current flows (reset)
#define HC2_UBAT_DIFF_MAX_SET      0.3 // Maximum difference between Ubat and voltage at HC2_GND for detection of a short to Ubat (set)
#define HC2_UBAT_DIFF_MAX_RESET    0.1 // Maximum difference between Ubat and voltage at HC2_GND for detection of a short to Ubat (reset)
#define HC2_UGND_MAX_SET           0.2// Maximum voltage at HC2_GND for detection of a short to GND (set)
#define HC2_UGND_MAX_RESET         0.1 // Maximum voltage at HC2_GND for detection of a short to GND (reset)
#define HC2_ADC_DEBOUNCE_MS       100  // Debounce time for all errors detected by ADC at HC2_GND
#define HC2_DIAGSTIMUL_FIRST_MS  5000  // Delay after reset before first stimulus of diagnostics (Highside ON phase)
#define HC2_DIAGSTIMUL_OTHER_MS  5000  // Delay between stimulus of diagnostics (Highside ON phase)
#define HC2_DIAGSTIMUL_TIME_MS   1000  // Activation time of stimulus for diagnostics (Highside ON phase)

#define HC2_ST1_P1_T_MAIN_MS      50   // Duration of mainflash in ms: 0..255*TIMER_CYLE_MS
#define HC2_ST1_P2_Q_PRE           2   // Quocient duration preflash/mainflash: pre=main/q_pre
#define HC2_ST1_P3_T_PAUSE       300   // Pause between two pulsegroups in ms: 0..255*TIMER_CYLE_MS
#define HC2_ST1_P4_F_MAIN          9.5 // Factor for intensity of mainflash: allowed range 1-26.5 in steps of 0.1
#define HC2_ST1_P5_F_PREF          0.85// Factor for intensity of preflash from mainflash: allowed range 0.01..1.00 in steps of 0.01
#define HC2_ST1_P6_N_REPS          4   // Number of repetitions for the pulsegroups: allowed range 0..255
#define HC2_ST1_P7_T_IDLE_MS    1500   // Duration with basic brightness until start of next flashsequence: 0..255*TIMER_CYLE_MS

#define HC2_ST2_P1_T_MAIN_MS      50   // Duration of mainflash in ms: 0..255*TIMER_CYLE_MS
#define HC2_ST2_P2_Q_PRE           3   // Quocient duration preflash/mainflash: pre=main/q_pre
#define HC2_ST2_P3_T_PAUSE       300   // Pause between two pulsegroups in ms: 0..255*TIMER_CYLE_MS
#define HC2_ST2_P4_F_MAIN          9.5 // Factor for intensity of mainflash: allowed range 1-26.5 in steps of 0.1
#define HC2_ST2_P5_F_PREF          0.85// Factor for intensity of preflash from mainflash: allowed range 0.01..1.00 in steps of 0.01
#define HC2_ST2_P6_N_REPS          4   // Number of repetitions for the pulsegroups: allowed range 0..255
#define HC2_ST2_P7_T_IDLE_MS    1500   // Duration with basic brightness until start of next flashsequence: 0..255*TIMER_CYLE_MS

#define HC2_ST3_P1_T_MAIN_MS      50   // Duration of mainflash in ms: 0..255*TIMER_CYLE_MS
#define HC2_ST3_P2_Q_PRE           1   // Quocient duration preflash/mainflash: pre=main/q_pre
#define HC2_ST3_P3_T_PAUSE       300   // Pause between two pulsegroups in ms: 0..255*TIMER_CYLE_MS
#define HC2_ST3_P4_F_MAIN          9.5 // Factor for intensity of mainflash: allowed range 1-26.5 in steps of 0.1
#define HC2_ST3_P5_F_PREF          0.85// Factor for intensity of preflash from mainflash: allowed range 0.01..1.00 in steps of 0.01
#define HC2_ST3_P6_N_REPS          4   // Number of repetitions for the pulsegroups: allowed range 0..255
#define HC2_ST3_P7_T_IDLE_MS    1500   // Duration with basic brightness until start of next flashsequence: 0..255*TIMER_CYLE_MS

// UK Change
#define HC2_P8_RAMP_UP_MS         10    // Brightness ramp up time in ms for HC2 ON 
#define HC2_P9_RAMP_DOWN_MS       10    // Brightness ramp down time in ms for HC2 OFF 

// Paramteres for Temperature Monitoring
#define TMON_MCU_OVT_SET         125.  // Temperature in °C to set the OVT-error
#define TMON_MCU_OVT_RESET       115.  // Temperature in °C to reset the OVT-error

// Parameters for Voltage Monitoring
#define VMON_UV_SET_DELAYED         8.5 // Threshold for setting undervoltage condition
#define VMON_UV_SET_DLY_MS       5000   // Delay for detection undervoltage condition in ms
#define VMON_UV_RESET_DELAYED       9.0 // Threshold for reset of undervoltage condition
#define VMON_UV_RESET_DLY_MS     1500   // Delay for reset of undervoltage condition in ms

#define VMON_OV_SET_IMMEDIATE      18.  // Threshold for immediate setting of overvoltage condition
#define VMON_OV_SET_DELAYED        16.4 // Threshold for delayed setting of overvoltage condition
#define VMON_OV_SET_DLY_MS       1500   // Delay for detection of overvoltage condition in ms (at least 200ms)
#define VMON_OV_RESET_DELAYED      16.  // Threshold for reset of overvoltage condition
#define VMON_OV_RESET_DLY_MS     1500   // Delay for reset of overvoltage condition

#define VMON_UV_HTR_SET_DELAYED    10.8 // Threshold for setting heater undervoltage condition
#define VMON_UV_HTR_SET_DLY_MS   5000   // Delay for detection heater undervoltage condition in ms
#define VMON_UV_HTR_RESET_DELAYED  11.6 // Threshold for reset of heater undervoltage condition
#define VMON_UV_HTR_RESET_DLY_MS 5000   // Delay for reset of heater undervoltage condition in ms

/*  possibly an extra monitoring for diagnostic function range
#define VMON_DIAG_OK_MIN_SET_DELAYED 11. // Low voltage limit for diagnostic functions
#define VMON_DIAG_OK_MAX_SET_DELAYED 16. // High voltage limit for diagnostic functions
#define VMON_DIAG_OK_SET_DLY_MS    5000  // Time for voltage in range before diagnostics allowed
*/

 

// Scheduling of application call cycles
#define VMON_CYCLE_TIME_MS MAIN_CYCLE_MS
#define MMOT_CYCLE_TIME_MS MAIN_CYCLE_MS
#define MPOT_CYCLE_TIME_MS MAIN_CYCLE_MS
#define PFLD_CYCLE_TIME_MS MAIN_CYCLE_MS
#define HTR_CYCLE_TIME_MS  MAIN_CYCLE_MS
#define EC_CYCLE_TIME_MS   MAIN_CYCLE_MS
#define HC2_CYCLE_TIME_MS  MAIN_CYCLE_MS



#define PFLD_DRIVE     eHB1 // + when moving to driving position
#define PFLD_PARK      eHB2 // + when moving to parking position

// Values for BRC_CTR_EXMI_FMIR_LIN
#define LIN_CTRL_PFLD_NO    0
#define LIN_CTRL_PFLD_DRIVE 2
#define LIN_CTRL_PFLD_PARK  1
#define LIN_CTRL_PFLD_SNV   3
  


#endif // _xxx_H_
/*-- End of File --*/

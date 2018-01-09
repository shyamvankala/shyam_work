 
#ifndef _PFLD_H_
#define _PFLD_H_



#define PFLD_STATE_IDLE     0 // nothing active
#define PFLD_STATE_RUNNING  1 // motor on, travelling between the two positions
#define PFLD_STATE_BLOCKING 2 // motor on, block current detected, switch-off timer running
#define PFLD_STATE_WAITING  3 // wait for change or release of LIN-command
#define PFLD_STATE_BRAKING  4 // Both terminals at the same potential (Ubat or GND)
#define PFLD_STATE_SUSPEND  5 // motor suspended (from PFLD_STATE_RUNNING)

// #define PFLD_MOVE_DRIVE()         HB_vHBControl(eHB1,eHB_LON_HOFF); HB_vHBControl(eHB2,eHB_LOFF_HON);
//#define PFLD_MOVE_PARK()          HB_vHBControl(eHB2,eHB_LON_HOFF);	HB_vHBControl(eHB1,eHB_LOFF_HON);
//#define PFLD_BRAKE()              HB_vHBControl(eHB2,eHB_LON_HOFF); HB_vHBControl(eHB1,eHB_LON_HOFF);


#define PFLD_DIR_DRIVE 0
#define PFLD_DIR_PARK  1
#define PFLD_DIR_OFF   2 // used for OFF and invalid only as return value from function
/* Type-definitions #############################################################################################################*/
typedef union{
  uint8_t byte;
  struct {
    uint8_t state       :3; // actual state
    uint8_t dir         :1; // direction
    uint8_t dir_change  :1; // direction change during movement
    uint8_t repeat      :1; // repeatation in case of low supply voltage
  }sub;
}typePfldState;


 #define PFLD_CYCLE_TIME_MS MAIN_CYCLE_MS
// #define PFLD_PEAK_TIME_MS         50  
 #define PFLD_BLOCK_TIME_MS   		500
 #define PFLD_MAX_ON_TIME 			1500


extern typePfldState PfldState;
#define PfldGetState() PfldState
#define PfldGetDirection() PfldGetState().sub.dir
#define PfldIsActive() ((PfldGetState().sub.state==PFLD_STATE_RUNNING) || (PfldGetState().sub.state==PFLD_STATE_BLOCKING))
#define PfldIsBraking() (PfldGetState().sub.state == PFLD_STATE_BRAKING)

/////////////////////////////////////////////////// Powerfold ////////////////////////////////////////////////////////////////////
#define PFLD_OVERCURRENT()        u8HBOutputState&0x03  // BR the outputstate variable contains the values of HBDSTAT and 0 & 1 bit represent Overcurrent in HB1,HB2   
#define PFLD_OPEN_LOAD_DRIVE()    HB_eGetHBError(eHB1) == eHB_OpenLoad//((gL99StatReg2.word&(L99_HBR_IN_HS_MASK(PFLD_DRIVE)|L99_HBR_IN_LS_MASK(PFLD_PARK)))==(L99_HBR_IN_HS_MASK(PFLD_DRIVE)|L99_HBR_IN_LS_MASK(PFLD_PARK)))
#define PFLD_OPEN_LOAD_PARK()     HB_eGetHBError(eHB2) == eHB_OpenLoad//((gL99StatReg2.word&(L99_HBR_IN_LS_MASK(PFLD_DRIVE)|L99_HBR_IN_HS_MASK(PFLD_PARK)))==(L99_HBR_IN_LS_MASK(PFLD_DRIVE)|L99_HBR_IN_HS_MASK(PFLD_PARK)))
#define PFLD_OPEN_LOAD( dir)      (dir==PFLD_DIR_DRIVE?PFLD_OPEN_LOAD_DRIVE():PFLD_OPEN_LOAD_PARK())
void PfldCyclic( void);
/* Interface-functions ##########################################################################################################*/
extern void PfldInit( void);
extern void PfldTimerIsr( void);
void PfldMotorBrake(void);
 #endif

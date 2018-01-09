 /**********************************************************************************************************************************
Project     :  L7-Mirror
Filename    :  Mmot.h
Brief descr.:  Headerfile for Mmot (Mirror motor) functions (control and diagnostics)
Copyright   :  
Remarks     :  
Version
  1.0.0  18.09.2009  UF  Creation
  1.0.1  17.01.2010  UF  Added interface for extended halfbridge diagnostics
**********************************************************************************************************************************/
#ifndef _MMOT_H_
#define _MMOT_H_

#include "stdint.h"
#include "std_init.h"
#include "error.h"
 #include "Applinit.h"
 
/* Include-Files ################################################################################################################*/
/* Defines ######################################################################################################################*/
// Values for MMot-Direction !!! if this assigment is changed, also MmDirToHsSwitchTbl[] and MmotIsDirectionX/Y must be adapted!
#define MMOT_DIR_LEFT   0 // Axis X, Bit 1 clear
#define MMOT_DIR_RIGHT  1 // Axis X, Bit 1 clear
#define MMOT_DIR_UP     2 // Axis Y, Bit 1 set
#define MMOT_DIR_DOWN   3 // Axis Y, Bit 1 set
#define MMOT_DIR_OFF    4 // used for OFF and invalid only as return value from function

// Values for typeMmState.sub.state
#define MMOT_STATE_IDLE     0 // nothing active
#define MMOT_STATE_RUNNING  1 // motor running
#define MMOT_STATE_BRAKING  2 // motor braking
#define MMOT_STATE_SUSPEND  3 // motor suspended (from MMOT_STATE_RUNNING)
#define MMOT_STATE_ERROR    4 // Error detected (Timeout or overcurrent)
//BR
#define MMOT_OPEN_LOAD()      (HB_eGetHBError(eHB3) == eHB_OpenLoad || HB_eGetHBError(eHB4) == eHB_OpenLoad)
#define MMOT_OVERCURRENT()     u8HBOutputState&0x0E     // BR the outputstate variable contains the values of HBDSTAT and 1,2,3 bit represent Overcurrent in HB2,HB3 & HB4  
/* Type-definitions #############################################################################################################*/
typedef union{
  uint8 byte;
  struct {
    uint8 state     :3; // idle/running/braking/suspended/timeout
    uint8 dir       :2; // direction
    uint8 memory    :1; // 0=manual, 1=memory
    uint8 mem_x_ok  :1; // Memory X finished
    uint8 mem_y_ok  :1; // Memory Y finished
    uint8 moved     :1; // Movement detected since motor start (important for poti-move detection timeout)
    uint8 error     :1; // Reset at start, set at error and important to enter correct state after braking is finished
    uint8 mem_rep_x :1; // For enabling the second memory cycle in X-Axis
    uint8 mem_rep_y :1; // For enabling the second memory cycle in Y-Axis
  }sub;
}typeMmotState;
extern uint8 Memory_Request;
/* Declarations #################################################################################################################*/
#ifdef _MMOT_C_
# define extern
#endif
/* Global variables #############################################################################################################*/
//#pragma space extern [] @tiny
extern typeMmotState MmotState;
//#pragma space extern [] @near
/* Interface-functions ##########################################################################################################*/
extern void MmotInit( void);
extern void MmotCyclic( void);
#if _DEBUG==ON
extern void MmotSetHbExtDiagErrorState( typeErrorState State);
#endif
#define MmotGetState() MmotState
#define MmotIsActive() (MmotGetState().sub.state == MMOT_STATE_RUNNING)
#define MmotIsBraking() (MmotGetState().sub.state == MMOT_STATE_BRAKING)
#define MmotGetDirection() MmotGetState().sub.dir
#define MmotIsDirectionX() (((MmotGetDirection())&0x02)==0)
#define MmotIsDirectionY() (((MmotGetDirection())&0x02)!=0)

#define MemRequest() Memory_Request
void ZMDIMmotHBBrake(void);
void MemoryStart( uint8 GotoX, uint8 GotoY);
void HB_DIAG(uint8_t value);
# if _DEBUG==ON // for debugging only  
extern const typeErrorState* MmotGetErrors( void);
# endif

#ifdef extern
# undef extern
#endif
#endif // _xxx_H_
/*-- End of File --*/

/*********************************************************************************************************************************//**********************************************************************************************************************************
Project     :  L7-Mirror
Filename    :  Vmon.h
Brief descr.:   Standard include file. Includes common used headerfiles.
Copyright   :  
Remarks     : 
Version

**********************************************************************************************************************************/




/* Include-Files ################################################################################################################*/
#include "std_init.h"
#include "ZMDCM0.h"
#include "SPI.h"
#include "ADC.h"
#include "TMR32.h"
#include "SBC_cfg.h"
#include "SBC.h"
//#include "TRIM.h"
#include "SPIMgr.h"
//#include "ADC_Appl.h"
#include "HB.h"
//#include "Debounce.h"				   


//typedef unsigned char         boolean; 


/*  Modification based on Application requirement*/

#define FALSE 0
#define TRUE 1

#define OFF 0
#define ON  1

#define LOW  0
#define HIGH 1

/* Type-definitions #############################################################################################################*/

typedef unsigned char         boolean;       /*        TRUE .. FALSE           */

typedef signed char           sint8;         /*        -127 .. +127            */
typedef signed char           int8;          /*        -127 .. +127            */
typedef unsigned char         uint8;         /*           0 .. 255             */
typedef signed short          sint16;        /*      -32767 .. +32767          */
typedef signed short          int16;         /*      -32767 .. +32767          */
typedef unsigned short        uint16;        /*           0 .. 65535           */
typedef signed long           sint32;        /* -2147483647 .. +2147483647     */
typedef signed long           int32;         /* -2147483647 .. +2147483647     */
typedef unsigned long         uint32;        /*           0 .. 4294967295      */
                                        
typedef signed char           sint8_least;   /* At least 7 bit + 1 bit sign    */
typedef unsigned char         uint8_least;   /* At least 8 bit                 */
typedef signed short          sint16_least;  /* At least 15 bit + 1 bit sign   */
typedef unsigned short        uint16_least;  /* At least 16 bit                */
typedef signed long           sint32_least;  /* At least 31 bit + 1 bit sign   */
typedef unsigned long         uint32_least;  /* At least 32 bit                */
                                        
typedef float                 float32;
typedef double                float64;

typedef uint8 Std_ReturnType;

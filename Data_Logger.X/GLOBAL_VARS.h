/* 
 * File:   GLOBAL_VARS.h
 * Author: Brian
 *
 * Created on June 22, 2016, 12:36 PM
 */
#include"GenericTypeDefs.h"
#include "SystemTimer.h"
#include"I2C_HardwareDrvr.h"

#ifndef GLOBAL_VARS_H
#define	GLOBAL_VARS_H

#ifdef	__cplusplus
extern "C" {
#endif

extern UINT InterruptCount;
extern UINT IntStatus;
extern UINT8 NumInstalledAccels; //Number of accelerometers installed
extern volatile UINT8 InterruptStatus;
extern volatile UINT8 DataReady1; //Flags for accelerometer interrupts during current reads (so we dont loose the interrupt)
extern volatile UINT8 DataReady2; //Flags for accelerometer interrupts during current reads (so we dont loose the interrupt)
extern volatile UINT8 Accel1ReadStarted; 
extern volatile UINT8 Accel2ReadStarted;
extern UINT8 InitComplete; 
extern volatile BOOL I2CBusCollision;
extern volatile UINT8 ucDataArray[10];
extern volatile UINT8 Temp_INT_SOURCE_REG;
extern volatile struct msTimer msTestCycleTimer, msLogTimer1, msLogTimer2;


#ifdef	__cplusplus
}
#endif

#endif	/* GLOBAL_VARS_H */


/* 
 * File:   GLOBAL_VARS.h
 * Author: Brian
 *
 * Created on June 22, 2016, 12:36 PM
 */
#include"GenericTypeDefs.h"
#ifndef GLOBAL_VARS_H
#define	GLOBAL_VARS_H

#ifdef	__cplusplus
extern "C" {
#endif

extern volatile UINT8 DataReady1; //Flags for accelerometer interrupts
extern volatile UINT8 DataReady2; //Flags for accelerometer interrupts

#ifdef	__cplusplus
}
#endif

#endif	/* GLOBAL_VARS_H */


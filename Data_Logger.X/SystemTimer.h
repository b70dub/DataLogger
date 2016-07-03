/* 
 * File:   TimeStamp.h
 * Author: Brian
 *
 * Created on June 9, 2016, 9:33 AM
 */

#include "GenericTypeDefs.h"

#ifndef TIMESTAMP_H
#define	TIMESTAMP_H

#ifdef	__cplusplus
extern "C" {
#endif

/******************************************************************************
* Sytem Timer - typedefs/structs
******************************************************************************/
typedef struct TimeStamp {
  BYTE year;
  BYTE month;
  BYTE day;
  BYTE hr;
  BYTE min;
  BYTE sec;
  int msec;

};
//extern struct TimeStamp TNow;

typedef struct Timer {
    struct TimeStamp StartTime;
    struct TimeStamp Setpt;
    struct TimeStamp RemainingTime;
    BOOL TimerComplete;
};
//extern struct Timer TestCycleTimer,LogTimer1,LogTimer2;

/******************************************************************************
* Sytem Timer - variables and constants
******************************************************************************/
//volatile int irtc_mSec;
///volatile BYTE rtcYear = 111, rtcMon = 11, rtcMday = 22;    // RTC date values
//volatile BYTE rtcHour = 0, rtcMin = 0, rtcSec = 0;    // RTC time values
//volatile unsigned long tick;                               // Used for ISR

/******************************************************************************
* Sytem Timer - functions
******************************************************************************/
void func_InitializeTime(struct TimeStamp* TimeNow);
void Func_UpdateSystemTime(struct TimeStamp* TimeNow, BYTE rtcYear, BYTE rtcMon, BYTE rtcMday, BYTE rtcHour, BYTE rtcMin, BYTE rtcSec, int irtc_mSec);
void func_GetRemainingTime(struct Timer* ThisTimer, struct TimeStamp* TimeNow);

#ifdef	__cplusplus
}
#endif

#endif	/* TIMESTAMP_H */


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
typedef struct rtcTimeStamp {
  BYTE year;
  BYTE month;
  BYTE day;
  BYTE hr;
  BYTE min;
  BYTE sec;
  int msec;

};

typedef struct rtcTimer {
    struct rtcTimeStamp StartTime;
    struct rtcTimeStamp Setpt;
    struct rtcTimeStamp RemainingTime;
    BOOL TimerComplete;
};

typedef struct msTimer {
    int StartTime;
    int Setpt;
    int RemainingTime;
    BOOL TimerComplete;
};

/******************************************************************************
* Sytem Timer - variables and constants
******************************************************************************/
//volatile int irtc_mSec;
///volatile BYTE rtcYear = 111, rtcMon = 11, rtcDay = 22;    // RTC date values
//volatile BYTE rtcHour = 0, rtcMin = 0, rtcSec = 0;    // RTC time values
//volatile unsigned long tick;                               // Used for ISR

/******************************************************************************
* Sytem Timer - functions
******************************************************************************/
void func_InitializeTime(struct rtcTimeStamp* TimeNow);
void Func_UpdateSystemTime(struct rtcTimeStamp* TimeNow, BYTE rtcYear, BYTE rtcMon, BYTE rtcDay, BYTE rtcHour, BYTE rtcMin, BYTE rtcSec, int irtc_mSec);
void func_GetRemainingTime_rtc(struct Timer* ThisTimer, struct rtcTimeStamp* TimeNow);

#ifdef	__cplusplus
}
#endif

#endif	/* TIMESTAMP_H */


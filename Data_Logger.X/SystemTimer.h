/* 
 * File:   TimeStamp.h
 * Author: Brian
 *
 * Created on June 9, 2016, 9:33 AM
 */

#ifndef TIMESTAMP_H
#define	TIMESTAMP_H

#ifdef	__cplusplus
extern "C" {
#endif


typedef struct TimeStamp {
  WORD year;
  WORD month;
  WORD day;
  WORD hr;
  WORD min;
  WORD sec;
  int msec;

} volatile TNow;

typedef struct Timer {
    struct TimeStamp StartTime;
    volatile struct TimeStamp Setpt;
    BOOL TimerComplete;
} TestCycleTimer, *ptrTestCycleTimer = &TestCycleTimer, LogTimer1, *ptrLogTimer1 = &LogTimer1, LogTimer2, *ptrLogTimer2 = &LogTimer2;

/******************************************************************************
* Sytem Timer - Global variables and constants
******************************************************************************/
volatile int irtc_mSec;
volatile BYTE rtcYear = 111, rtcMon = 11, rtcMday = 22;    // RTC date values
volatile BYTE rtcHour = 0, rtcMin = 0, rtcSec = 0;    // RTC time values
volatile unsigned long tick;                               // Used for ISR

#ifdef	__cplusplus
}
#endif

#endif	/* TIMESTAMP_H */


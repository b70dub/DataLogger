/*-----------------------------------------------------------------------------
 *
 * File: SystemTimer.c
 * Author: Brian Ankeny
 * PIC: 32MX440F256H @ 80MHz, 3.3v
 * MPLAB X IDE v3.35  
 *
 *-----------------------------------------------------------------------------
*/
#include "SystemTimer.h"

//This function initializes the current time (TNow) structure
void func_InitializeTime(struct rtcTimeStamp* TimeNow){
    TimeNow->year = 0;
    TimeNow->month = 0;
    TimeNow->day = 0;
    TimeNow->hr = 0;
    TimeNow->min = 0;
    TimeNow->sec = 0;
    TimeNow->msec = 0;
}

/*********************************************************************
 * Function:        DWORD get_fattime(void)
 * PreCondition:
 * Input:           None
 * Output:          Time
 * Side Effects:
 * Overview:     This function updates the TNow structure with the current values
 * Note:
 ********************************************************************/
void Func_UpdateSystemTime(struct rtcTimeStamp* TimeNow, BYTE rtcYear, BYTE rtcMon, BYTE rtcDay, BYTE rtcHour, BYTE rtcMin, BYTE rtcSec, int irtc_mSec){

   if (TimeNow->msec != irtc_mSec) {
      TimeNow->msec = irtc_mSec;
      if (TimeNow->sec != rtcSec) {
         TimeNow->sec = rtcSec;
         if (TimeNow->min != rtcMin) {
            TimeNow->min = rtcMin;
            if (TimeNow->hr != rtcHour) {
               TimeNow->hr = rtcHour;
               if (TimeNow->day != rtcDay) {
                  TimeNow->day = rtcDay;
                  if (TimeNow->month != rtcMon) {
                     TimeNow->month = rtcMon;
                     if (TimeNow->year != rtcYear) {
                        TimeNow->year = rtcYear;
                     }
                  }
               }
            }
         }
      }
   }

}


//This function will return how much time (in milliseconds) has passed for a given setpoint and start time
void func_GetRemainingTime_ms(struct msTimer* ThisTimer, int CurrentCount){
    

    ThisTimer->RemainingTime = ThisTimer->Setpt - (CurrentCount - ThisTimer->StartTime);

    if(ThisTimer->RemainingTime > 0)
        ThisTimer->TimerComplete = FALSE;
    else
        ThisTimer->TimerComplete = TRUE;

}

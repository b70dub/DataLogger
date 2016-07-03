#include "SystemTimer.h"

//This function initializes the current time (TNow) structure
void func_InitializeTime(struct TimeStamp* TimeNow){
    TimeNow->year = 0;
    TimeNow->month = 0;
    TimeNow->day = 0;
    TimeNow->hr = 0;
    TimeNow->min = 0;
    TimeNow->sec = 0;
    TimeNow->msec = 0;
}
/*
struct Timer Func_StartTimer(struct Timer ThisTimer){
    ThisTimer->StartTime.year    = TNow.year;
    ThisTimer->StartTime.month   = TNow.month;
    ThisTimer->StartTime.day     = TNow.day;
    ThisTimer->StartTime.hr      = TNow.hr;
    ThisTimer->StartTime.min     = TNow.min;
    ThisTimer->StartTime.sec     = TNow.sec;
    ThisTimer->StartTime.msec    = TNow.msec;

    return ThisTimer;
}
*/
/*********************************************************************
 * Function:        DWORD get_fattime(void)
 * PreCondition:
 * Input:           None
 * Output:          Time
 * Side Effects:
 * Overview:     This function updates the TNow structure with the current values
 * Note:
 ********************************************************************/
void Func_UpdateSystemTime(struct TimeStamp* TimeNow, BYTE rtcYear, BYTE rtcMon, BYTE rtcMday, BYTE rtcHour, BYTE rtcMin, BYTE rtcSec, int irtc_mSec){

   if (TimeNow->msec != irtc_mSec) {
      TimeNow->msec = irtc_mSec;
      if (TimeNow->sec != rtcSec) {
         TimeNow->sec = rtcSec;
         if (TimeNow->min != rtcMin) {
            TimeNow->min = rtcMin;
            if (TimeNow->hr != rtcHour) {
               TimeNow->hr = rtcHour;
               if (TimeNow->day != rtcMday) {
                  TimeNow->day = rtcMday;
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


//This funtion will return how much time has passed for a given setpoint and start time
void func_GetRemainingTime(struct Timer* ThisTimer, struct TimeStamp* TimeNow){
//void func_GetRemainingTime(struct Timer ThisTimer){

    ThisTimer->RemainingTime.year = TimeNow->year - (ThisTimer->StartTime.year + ThisTimer->Setpt.year);
    ThisTimer->RemainingTime.month = TimeNow->month - (ThisTimer->StartTime.month + ThisTimer->Setpt.month);
    ThisTimer->RemainingTime.day = TimeNow->day - (ThisTimer->StartTime.day + ThisTimer->Setpt.day);
    ThisTimer->RemainingTime.hr = TimeNow->hr - (ThisTimer->StartTime.hr + ThisTimer->Setpt.hr);
    ThisTimer->RemainingTime.min = TimeNow->min - (ThisTimer->StartTime.min + ThisTimer->Setpt.min);
    ThisTimer->RemainingTime.sec = TimeNow->sec - (ThisTimer->StartTime.sec + ThisTimer->Setpt.sec);
    ThisTimer->RemainingTime.msec = TimeNow->msec - (ThisTimer->StartTime.msec + ThisTimer->Setpt.msec);

    if((ThisTimer->RemainingTime.msec > 0) || (ThisTimer->RemainingTime.sec > 0)|| (ThisTimer->RemainingTime.min > 0) || (ThisTimer->RemainingTime.hr > 0) || (ThisTimer->RemainingTime.day > 0) || (ThisTimer->RemainingTime.month > 0) || (ThisTimer->RemainingTime.year > 0))
        ThisTimer->TimerComplete = FALSE;
    else
        ThisTimer->TimerComplete = TRUE;

}

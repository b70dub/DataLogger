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
/*
struct rtcTimer Func_StartTimer(struct rtcTimer ThisTimer){
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
void Func_UpdateSystemTime(struct rtcTimeStamp* TimeNow, BYTE rtcYear, BYTE rtcMon, BYTE rtcMday, BYTE rtcHour, BYTE rtcMin, BYTE rtcSec, int irtc_mSec){

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

/*
//This funtion will return how much time has passed for a given setpoint and start time
void func_GetRemainingTime_rtc(struct rtcTimer* ThisTimer, struct rtcTimeStamp* TimeNow){
    BYTE Temp_year, Temp_month, Temp_day, Temp_hour, Temp_min, Temp_sec;
    int Temp_msec;

    
    Temp_year = ThisTimer->Setpt.year -  (TimeNow->year - ThisTimer->StartTime.year);
    
    Temp_month = ThisTimer->Setpt.month - (TimeNow->month - ThisTimer->StartTime.month);
    Temp_day = ThisTimer->Setpt.day - (TimeNow->day - ThisTimer->StartTime.day);
    Temp_hour = ThisTimer->Setpt.hr - (TimeNow->hr - ThisTimer->StartTime.hr);
    Temp_min = ThisTimer->Setpt.min - (TimeNow->min - ThisTimer->StartTime.min);
    Temp_sec = ThisTimer->Setpt.sec - (TimeNow->sec - ThisTimer->StartTime.sec);
    
    Temp_msec = ThisTimer->Setpt.msec - (TimeNow->msec - ThisTimer->StartTime.msec);
    
    if((Temp_msec < 0) && (Temp_sec > 0))
    {
        Temp_sec--;
        Temp_msec = 1000 + Temp_msec;
    }
    
    else if((Temp_sec < 0) && (Temp_min > 0))
    {
        Temp_min--;
        Temp_sec = 60 + Temp_sec;
    }
    
    else if((Temp_min < 0) && (Temp_hour > 0))
    {
        Temp_hour--;
        Temp_min = 60 + Temp_min;
    }
      

    ThisTimer->RemainingTime.year = 
    ThisTimer->RemainingTime.month =
    ThisTimer->RemainingTime.day =
    ThisTimer->RemainingTime.hr =  
    ThisTimer->RemainingTime.min =        
    ThisTimer->RemainingTime.msec =
    

    if((ThisTimer->RemainingTime.msec > 0) || (ThisTimer->RemainingTime.sec > 0)|| (ThisTimer->RemainingTime.min > 0) || (ThisTimer->RemainingTime.hr > 0) || (ThisTimer->RemainingTime.day > 0) || (ThisTimer->RemainingTime.month > 0) || (ThisTimer->RemainingTime.year > 0))
        ThisTimer->TimerComplete = FALSE;
    else
        ThisTimer->TimerComplete = TRUE;

}
*/

//This function will return how much time (in milliseconds) has passed for a given setpoint and start time
void func_GetRemainingTime_ms(struct msTimer* ThisTimer, int CurrentCount){
    

    ThisTimer->RemainingTime = ThisTimer->Setpt - (CurrentCount - ThisTimer->StartTime);

    if(ThisTimer->RemainingTime > 0)
        ThisTimer->TimerComplete = FALSE;
    else
        ThisTimer->TimerComplete = TRUE;

}

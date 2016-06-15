//This function initializes the current time (TNow) structure
struct TimeStamp func_InitializeTime(struct TimeStamp TimeNow){
    TimeNow.year = 0;
    TimeNow.month = 0;
    TimeNow.day = 0;
    TimeNow.hr = 0;
    TimeNow.min = 0;
    TimeNow.sec = 0;
    TimeNow.msec = 0;
            
    return TimeNow;
}

//This function updates the TNow structure with the current values
void Func_UpdateSystemTime(){

   if (TNow.msec != irtc_mSec) {
      TNow.msec = irtc_mSec;
      if (TNow.sec != rtcSec) {
         TNow.sec = rtcSec;
         if (TNow.min != rtcMin) {
            TNow.min = rtcMin;
            if (TNow.hr != rtcHour) {
               TNow.hr = rtcHour;
               if (TNow.day != rtcMday) {
                  TNow.day = rtcMday;
                  if (TNow.month != rtcMon) {
                     TNow.month = rtcMon;
                     if (TNow.year != rtcYear) {
                        TNow.year = rtcYear;
                     }
                  }
               }
            }
         }
      }
   }

}

//This funtion will return how much time has passed for a given setpoint and start time
//struct TimeStamp func_GetRemainingTime(struct Timer* ThisTimer){
void func_GetRemainingTime(struct Timer* ThisTimer){
    struct TimeStamp  RemainingTime;

    RemainingTime.year = TNow.year - (&ThisTimer.StartTime.year + &ThisTimer.Setpt.year);
    RemainingTime.month = TNow.month - (&ThisTimer.StartTime.month + &ThisTimer.Setpt.month);
    RemainingTime.day = TNow.day - (&ThisTimer.StartTime.day + &ThisTimer.Setpt.day);
    RemainingTime.hr = TNow.hr - (&ThisTimer.StartTime.hr + &ThisTimer.Setpt.hr);
    RemainingTime.min = TNow.min - (&ThisTimer.StartTime.min + &ThisTimer.Setpt.min);
    RemainingTime.sec = TNow.sec - (&ThisTimer.StartTime.sec + &ThisTimer.Setpt.sec);
    RemainingTime.msec = TNow.msec - (&ThisTimer.StartTime.msec + &ThisTimer.Setpt.msec);

    if((RemainingTime.msec > 0) || (RemainingTime.sec > 0)|| (RemainingTime.min > 0) || (RemainingTime.hr > 0) || (RemainingTime.day > 0) || (RemainingTime.month > 0) || (RemainingTime.year > 0))
        &ThisTimer.TimerComplete = FALSE;
    else
        &ThisTimer.TimerComplete = TRUE;

   // return RemainingTime;
}

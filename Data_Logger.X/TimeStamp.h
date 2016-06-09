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

} TNow;

typedef struct Timer {
    struct TimeStamp StartTime;
    struct TimeStamp Setpt;
} AccReadDelay;


#ifdef	__cplusplus
}
#endif

#endif	/* TIMESTAMP_H */


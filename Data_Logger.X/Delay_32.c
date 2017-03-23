/*
* File: Delay_32.c
* PIC: 32MX440F256H @ 80MHz, 3.3v
* Compiler: MPLAB X IDE v3.35
* Program Description: This file demonstrates logging data to an SD card that is
*                      read from two MMA8452 accelerometers
*            
*
* Modified From: from microchip file "CoreTimerDelay.c"
*               All Rights belong to their respective owners.
* Tested on: 32MX440F256H @ 80MHz
*/

#include "Delay_32.h"

#define GetSystemClock()        (80000000ul)
#define us_SCALE   (GetSystemClock()/2000000)
#define ms_SCALE   (GetSystemClock()/2000)

/*******************************************************************************
* Function: void delay_ms (unsigned long int msDelay)
*
* Returns: Nothing
*
* Description:   Millisecond Delay function using the Count register
*                in coprocessor 0 in the MIPS core.
*
*******************************************************************************/

void delay_ms (unsigned long int msDelay )
{
      register unsigned int startCntms = ReadCoreTimer();
      register unsigned int waitCntms = msDelay * ms_SCALE;

      while( ReadCoreTimer() - startCntms < waitCntms );
}

/*******************************************************************************
* Function: void delay_ms (unsigned long int usDelay)
*
* Returns: Nothing
*
* Description: Microsecond Delay function using the Count register
*              in coprocessor 0 in the MIPS core.
*
*******************************************************************************/

void delay_us(unsigned long int usDelay )
{
      register unsigned int startCnt = ReadCoreTimer();
      register unsigned int waitCnt = usDelay * us_SCALE;

      while( ReadCoreTimer() - startCnt < waitCnt );
}
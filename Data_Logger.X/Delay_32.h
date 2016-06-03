/*******************************************************************************
 * File:   Delay_32.h
 * Author: Armstrong Subero
 * PIC: 32MX230F064B @ 40MHz, 3.3v
 * Program: Header file for MX230F064B delay routines
 * Compiler: XC32 (v1.33, MPLAX X v2.30)
 * Program Version 1.0
 * Program Description: This header file constains MX230F064B delay functions
 * Modified From: Microchip Forum discussion
 * Created on February 8, 2015, 2:40 PM
 ******************************************************************************/


/*******************************************************************************
 Includes and defines
*******************************************************************************/

#include <plib.h>

/*********************************************************************
* Macro: #define GetSystemClock() returns system clock frequency in Hertz. Value is 8 MHz/2 x 18 PLL for PIC32
*
* Macro: #define GetPeripheralClock() returns peripheral clock frequency.
*
* Macro: #define GetInstructionClock() returns instruction clock frequency.
*
********************************************************************/

//#define GetSystemClock() (40000000UL)/* Fcy = 40MHz */
#define GetSystemClock()        (80000000ul) //pic32 runs at 80mHz
#define GetPeripheralClock()    (GetSystemClock() / (1 << OSCCONbits.PBDIV))
#define GetInstructionClock()   (GetSystemClock())

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
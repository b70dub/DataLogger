/****************************************************************************
 * File:   Delay_32.h
 * Author: Brian Ankeny
 * PIC: 32MX440F256H @ 80MHz, 3.3v
 * Compiler: XC32 (v1.4, MPLAB X v2.20)
 * Program Version 0.0.0.1
 * Program Description: Delay functions for 32MX440F256H
 * Modified From: Microchip Forum discussion
 * Updated June 23, 2016
 ****************************************************************************/


/*****************************************************************************
 Includes and defines
******************************************************************************/

#include <plib.h>
#include <peripheral/timer.h>

void delay_ms (unsigned long int msDelay );
void delay_us(unsigned long int usDelay );



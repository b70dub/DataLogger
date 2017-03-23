/*-----------------------------------------------------------------------------
 *
 * File: HardwareProfile.h
 * Author: Brian Ankeny
 * PIC: 32MX440F256H @ 80MHz, 3.3v
 * MPLAB X IDE v3.35  
 *
 *-----------------------------------------------------------------------------
*/



/*******************************************************************************
 Includes and defines
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>


#define _SUPPRESS_PLIB_WARNING

/*******************************************************************************
 Configuration settings
*******************************************************************************/

// DEVCFG3
// USERID = No Setting

#pragma config USERID =     0xffff

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)                      
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier, carry to 80MHz)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 2, run at 40MHz) 
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)


// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (Primary osc XT)        
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_2           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/2 = 40 mhz)           
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1              // Watchdog Timer Postscaler (1:1048576)         
//#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))


//   Comm Channel Select:
//**     ICS_PGx3             Configure to use PGC3/PGD3
//**     ICS_PGx2             Configure to use PGC2/PGD2
//**     ICS_PGx1             Configure to use PGC1/PGD1

// DEVCFG0
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Pinguino Board uses PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)
#pragma config DEBUG    = OFF           // Debugger Disabled for Starter Kit                        -- BTA Added for I2C Comms originally did not exist


// Clock Settings

#define GetSystemClock()        (80000000ul)
#define GetInstructionClock()   (GetSystemClock())
#define TOGGLES_PER_SEC   1000
#define CORE_TICK_RATE   (GetSystemClock()/2/TOGGLES_PER_SEC)




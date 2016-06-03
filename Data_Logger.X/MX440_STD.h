/*
 * File:   MX230_STD.h
 * Author: Armstrong Subero
 * PIC: 32MX230F064B @ 40MHz, 3.3v
 * Program: Header file for configuring MX230F064B (STD use)
 * Compiler: XC32 (v1.33, MPLAX X v2.30)
 * Program Version 1.0
 * Program Description: This header file configures MX230F064B for standard use.
 *
 * Created on February 8, 2015, 3:59 PM
 */

/*******************************************************************************
 Includes and defines
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>


/*******************************************************************************
 Configuration settings
*******************************************************************************/

// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = OFF          // USB VBUS ON Selection (Controlled by Port Function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_1         // PLL Input Divider (1x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (16x Multiplier, carry to 80MHz)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2, run at 40MHz)
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)


// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (Primary osc XT)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))


//   Comm Channel Select:
//**     ICS_PGx3             Configure to use PGC3/PGD3
//**     ICS_PGx2             Configure to use PGC2/PGD2
//**     ICS_PGx1             Configure to use PGC1/PGD1

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Pinguino Board uses PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)




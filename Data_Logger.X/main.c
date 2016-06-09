/*******************************************************************************
* File: Main.c
* Author: Armstrong Subero
* PIC: 32MX230F064B @ 40MHz, 3.3v
* Program: PIC32 Based Data Logger
* Compiler: XC32 (v1.33, MPLABX v2.30)
* Created On: February 14th, 2015, 9:40 PM
* Description: This file demonstrates logging data to an SD card that is read
*               from an analog input channel on AN3 (PINB1)
*
* Modified From:NBIIFS from microchip forums, modified to run on PIC32MX230F064B
*               and log integer as well as text values.
*               All Rights belong to their respective owners.
* Dependencies: "NOKIA5110.h", "MX230_STD.h", "Delay_32.h", "Bitmaps.h"
* Tested on: 32MX230F064B @ 40MHz
* Version: 1.0
*
* An SD card (sparkfun SD/MMC breakout) is connected to
*                     the microcontroller in the following configuration:
*
*                    D2         --> NC
*                    D3(SS)     --> RB0 (PIN4)(pullup to Vcc)
*                    CMD(MOSI)  --> SDO1 (PIN17) (pullup to Vcc)
*                    CD         --> NC
*                    CLK(SCK)   --> SCK1 (PIN25)
*                    VCC        --> 3.3V (Decoupled, cap, tantalum)
*                    GND        --> GND  (Decoupled, cap, tantalum)
*                    D0(MISO)   --> SDI1 (PIN14)(pullup to Vcc)
*                    D1         --> NC
*                    WP         --> NC
*
* A Nokia5110 is connected to SPI2 and configuration is to be found in the
* 5110_SPI2.h header file
*
* A 4MHz crystal is connected to OSC1
*
* An LM34 temperature sensor is connected to PINB1
*/

/*******************************************************************************
 Includes and defines
 ******************************************************************************/
#include <plib.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "MX440_STD.h"
#include "Delay_32.h"
//#include "5110_SPI2.h"
#include "32_ADC.h"
#include "ff.h"
#include "TimeStamp.h"

//#define GetSystemClock() (40000000ul)
#define GetSystemClock()        (80000000ul) //pic32 runs at 80mHz
#define TOGGLES_PER_SEC   1000
#define CORE_TICK_RATE   (GetSystemClock()/2/TOGGLES_PER_SEC)

/*******************************************************************************
 Program global variables
 ******************************************************************************/
 float channel3;   //conversion result as read from result buffer
 float convert;    //variable for storing 10bit conversion result
 float farenheit;  //variable for storing farenheit result
 float celcius;    //variable for storing celcius result
 int   temp;       //final temperature will be cast into an integer

 int x;             //used for looping
 //int z = 0;         //count for lcd
 long values[1440]; //array for storing values
 long i;            //variable for storing values
 
 UINT len;          //needed for writing file

 static int irtc_mSec;
 volatile BYTE rtcYear = 111, rtcMon = 11, rtcMday = 22;    // RTC date values
 volatile BYTE rtcHour = 0, rtcMin = 0, rtcSec = 0;    // RTC time values
 volatile unsigned long tick;                               // Used for ISR

//Work registers for fs command
DWORD acc_size;         /* Work register for fs command */
WORD acc_files, acc_dirs;
FILINFO Finfo;
const BYTE ft[] = {0,12,16,32};

//File system object
FATFS Fatfs;
FATFS *fs;            /* Pointer to file system object */
FIL file1, file2;      /* File objects */

/*****************************************************************************
 * Function:        void CoreTimerHandler(void)
 * PreCondition:
 * Input:           None
 * Output:          None
 * Side Effects:
 * Overview:        FatFs requires a 1ms tick timer to aid
 *               with low level function timing
 * Note:            Initial Microchip version adapted to work into ISR routine
 *****************************************************************************/
void __ISR(_CORE_TIMER_VECTOR, IPL2SOFT) CoreTimerHandler(void)
{
   static const BYTE dom[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

   BYTE n;

   // clear the interrupt flag
   mCTClearIntFlag();
    // update the period
    UpdateCoreTimer(CORE_TICK_RATE);

   disk_timerproc();   // call the low level disk IO timer functions
   tick++;            // increment the benchmarking timer

   // implement a 'fake' RTCC
   if (++irtc_mSec >= 1000) {
      irtc_mSec = 0;
      if (++rtcSec >= 60) {
         rtcSec = 0;
         if (++rtcMin >= 60) {
            rtcMin = 0;
            if (++rtcHour >= 24) {
               rtcHour = 0;
               n = dom[rtcMon - 1];
               if ((n == 28) && !(rtcYear & 3)) n++;
               if (++rtcMday > n) {
                  rtcMday = 1;
                  if (++rtcMon > 12) {
                     rtcMon = 1;
                     rtcYear++;
                  }
               }
            }
         }
      }
   }
}


/*********************************************************************
 * Function:        DWORD get_fattime(void)
 * PreCondition:
 * Input:           None
 * Output:          Time
 * Side Effects:
 * Overview:     When writing fatfs requires a time stamp
 *               in this exmaple we are going to use a counter
 *               If the starter kit has the 32kHz crystal
 *               installed then the RTCC could be used instead
 * Note:
 ********************************************************************/
DWORD get_fattime(void)
{
   DWORD tmr;

   INTDisableInterrupts();
   tmr =     (((DWORD)rtcYear - 80) << 25)
         | ((DWORD)rtcMon << 21)
         | ((DWORD)rtcMday << 16)
         | (WORD)(rtcHour << 11)
         | (WORD)(rtcMin << 5)
         | (WORD)(rtcSec >> 1);
   INTEnableInterrupts();

   return tmr;
}

/*******************************************************************************
* Function: int main()
*
* Returns: Nothing
*
* Description: Program entry point, logs data for a 24 hour period then writes
*              it to a TEXT filei n an SD card
*
*******************************************************************************/

int main (void)
{

// Enable optimal performance
INTEnableSystemMultiVectoredInt();
SYSTEMConfigPerformance(GetSystemClock());
mOSCSetPBDIV(OSC_PB_DIV_1);            // Use 1:1 CPU Core:Peripheral clocks
OpenCoreTimer(CORE_TICK_RATE);         // Open 1 ms Timer


// set up the core timer interrupt with a prioirty of 2 and zero sub-priority
mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_2 | CT_INT_SUB_PRIOR_0));

SDI1Rbits.SDI1R = 1;        //SET SDI1 to RPB5 //PIN14 is SDI1
RPB8Rbits.RPB8R = 3;        //SET RPB8R to SDO1 //PIN17 is SD01

//Setup pin for input from AN3
TRISBbits.TRISB1 = 1;
PORTBbits.RB1 = 1;
ANSELBbits.ANSB1 = 1;

//Setup ADC conversion
adcON();

//Initlize LCD
//LCD5110_init();
//LCD5110_clearScreen();


/**********************Log Start***********************************************/
   //write data to lcd
   //LCD5110_send(0x40 + 0, 0); //Y address
   //LCD5110_send(0x80 + 0, 0); //X address
   //LCD5110_sendString ("Reading values");
   //delay_ms(1000);

    printf ((const char *)"Reading Values \r\n");
    //printf ((const rom far char *)"iInputValue = %d \r\n", iInputValue);

   //loop for a 24 hour period 60mins x 24hrs = 1440
   for (i = 0; i < 1440; i++){
       //read analog channel 3
      channel3 = ReadADC10(0);

      //do 10 bit conversion
      convert = (channel3 * 3300.0) / 1024;

      //convert to farenheit
      farenheit = convert/10.0;

      //convert to celcius
      celcius = (farenheit - 32) * 5/9;

      //cast celcius into integer for display
      temp = (int)celcius;

      //set the values in i to each time temp read
      values[i] = temp;

      //change this value to increase or decrease sampling speed
      //currently approx 1 min delay
      delay_ms(60000);
 }

 //set i back to 0
 i = 0;

//Alert user card being initilized
//LCD5110_clearScreen();
//LCD5110_send(0x40 + 0, 0); //Y address
//LCD5110_send(0x80 + 0, 0); //X address
//LCD5110_sendString ("Init SDCARD");
 printf ((const char *)"Init SDCARD \r\n");
delay_ms (50);

//Initialize Disk
disk_initialize(0);

//Aert user SD card is being mounted
//LCD5110_send(0x40 + 1, 0); //Y address
//LCD5110_send(0x80 + 0, 0); //X address
//LCD5110_sendString ("Mount SDCARD");
printf ((const char *)"Mount SDCARD \r\n");
delay_ms(50);

//Mount Filesystem
f_mount(0, &Fatfs);

//open data.txt file
const char *path = "0:data1.txt";
f_open(&file1, path, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);

//delay a little
delay_ms(100);

//Alert user data is being written
//LCD5110_send(0x40 + 2, 0); //Y address
//LCD5110_send(0x80 + 0, 0); //X address
//LCD5110_sendString ("Writing Data");

printf ((const char *)"Writing Data \r\n");

//write data to data1.txt
for (i = 0; i < 1440; i++){
    unsigned char buf[8];

    //convert values[] to ascii
    //itoa(buf, values[i], 10);


    //pointer to converted value
    //const char *text2 = buf;

    //convert values[] to ascii and pass back a pointer to the result string ---------------- BTA modified so test it!!!
    const char *text2 = itoa(values[i]);
    //write that value into text file
    f_write(&file1, text2, strlen(text2), &len);
 }


//Alert user file is closing
//LCD5110_send(0x40 + 3, 0); //Y address
//LCD5110_send(0x80 + 0, 0); //X address
//LCD5110_sendString ("Close File");
printf ((const char *)"Close File \r\n");

//Close data.txt
f_close(&file1);

//Alert user card is unmounting
//LCD5110_send(0x40 + 4, 0); //Y address
//LCD5110_send(0x80 + 0, 0); //X address
//LCD5110_sendString ("Unmount.Done");
printf ((const char *)"SD Card Unmounted \r\n");
//unmount filesystem
f_mount(0,NULL);

//delay some time
delay_ms(1000);

}



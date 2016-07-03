/*******************************************************************************
* File: Main.c
* Author: Brian Ankeny
* PIC: 32MX440F256H @ 80MHz, 3.3v
* Compiler: XC32 (v1.4, MPLAB X v2.20)
* Program Version 0.0.0.1
* Program Description: This file demonstrates logging data to an SD card that is
 *                      read from two MMA8452 accelerometers
*            
*
* Modified From:NBIIFS from microchip forums, modified to run on 32MX440F256H
*               and log integer as well as text values.
*               All Rights belong to their respective owners.
* Dependencies: "HardwareProfile.h", "Delay_32.h"
* Tested on: 32MX440F256H @ 80MHz

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
*/

/*******************************************************************************
 Includes and defines
 ******************************************************************************/
#include <plib.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "MMA8452_Config.h"
#include "HardwareProfile.h"

#include "ff.h"
#include "SystemTimer.h"
#include "GLOBAL_VARS.h"

//#define GetSystemClock() (40000000ul)
#define GetSystemClock()        (80000000ul) //pic32 runs at 80mHz
#define TOGGLES_PER_SEC   1000
#define CORE_TICK_RATE   (GetSystemClock()/2/TOGGLES_PER_SEC)

#define mPORTBClearBits(_bits) { LATB &= ~(unsigned int)_bits;}/*Clears PortB bit*/
#define mPORTCClearBits(_bits) { LATC &= ~(unsigned int)_bits;}/*Clears PortC bit*/
#define mPORTDClearBits(_bits) { LATD &= ~(unsigned int)_bits;}/*Clears PortD bit*/
#define mPORTEClearBits(_bits) { LATE &= ~(unsigned int)_bits;}/*Clears PortE bit*/
#define mPORTFClearBits(_bits) { LATF &= ~(unsigned int)_bits;}/*Clears PortF bit*/

#define mPORTBSetBits(_bits) { LATB |= (unsigned int)_bits;} /*Set PortB bit*/
#define mPORTCSetBits(_bits) { LATC |= (unsigned int)_bits;} /*Set PortC bit*/
#define mPORTDSetBits(_bits) { LATD |= (unsigned int)_bits;} /*Set PortD bit*/
#define mPORTESetBits(_bits) { LATE |= (unsigned int)_bits;} /*Set PortE bit*/
#define mPORTFSetBits(_bits) { LATF |= (unsigned int)_bits;} /*Set PortF bit*/


/*******************************************************************************
 Program global variables
 ******************************************************************************/
 int x;             //used for looping
 int iDeviceCount = 0;
 int iTempCount;

 //int z = 0;         //count for lcd
 long values[1440]; //array for storing values
 
 
 UINT len;          //needed for writing file
//volatile unsigned int
 volatile UINT8  TimeUpdated; //Flags for accelerometer interrupts

 /******************************************************************************
* Sytem Timer - variables and constants
******************************************************************************/
volatile int irtc_mSec;
volatile BYTE rtcYear = 111, rtcMon = 11, rtcMday = 22;    // RTC date values
volatile BYTE rtcHour = 0, rtcMin = 0, rtcSec = 0;    // RTC time values
volatile unsigned long tick;                               // Used for ISR

struct Timer TestCycleTimer,LogTimer1,LogTimer2;
struct TimeStamp TNow;


 //Accell data variables
 SHORT X1out_12_bit, Y1out_12_bit, Z1out_12_bit, X2out_12_bit, Y2out_12_bit, Z2out_12_bit;
 float X1out_g, Y1out_g, Z1out_g, X2out_g, Y2out_g, Z2out_g;


//Work registers for fs command
DWORD acc_size;         /* Work register for fs command */
WORD acc_files, acc_dirs;
FILINFO Finfo;
const BYTE ft[] = {0,12,16,32};

//File system object
FATFS Fatfs;
FATFS *fs;            /* Pointer to file system object */
FIL file1, file2;      /* File objects */



/* Interrupt service routine of external int 1    */
void __ISR(_EXTERNAL_1_VECTOR,IPL2SRS) INT1InterruptHandler(void)                   //Acellerometer 1 : Data Ready
{ 
 //LATDbits.LATD5 = 1;   // Set LED
 DataReady1 = 1;
 INTClearFlag(INT_INT1);       
 
} 
   
/* Interrupt service routine of external int 4    */
void __ISR(_EXTERNAL_4_VECTOR,IPL2SRS) INT4InterruptHandler(void)                   //Acellerometer 2 : Data Ready
{ 
 //LATDbits.LATD5 = 0;   // Clear LED
 DataReady2 = 1;
 INTClearFlag(INT_INT4);       
}

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

   TimeUpdated = 1;

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

/*********************************************************************
 * Function:        void Func_InitializeBoard(void)
 * PreCondition:
 * Input:           None
 * Output:          None
 * Side Effects:
 * Overview:     Used to initialize board parameters
 * Note:
 ********************************************************************/
void Func_InitializeBoard(){
    
    //Setup interrupts
 //removed temporarily
 //ConfigINT1(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_ENABLE); // Config INT1              //Acellerometer 1 : Data Ready

// ConfigINT4(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_ENABLE); // Config INT4              //Acellerometer 2 : Data Ready

 INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
 INTEnableInterrupts();

// Enable optimal performance
INTEnableSystemMultiVectoredInt();
SYSTEMConfigPerformance(GetSystemClock());
mOSCSetPBDIV(OSC_PB_DIV_1);            // Use 1:1 CPU Core:Peripheral clocks
OpenCoreTimer(CORE_TICK_RATE);         // Open 1 ms Timer


// set up the core timer interrupt with a priority of 2 and zero sub-priority
mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_2 | CT_INT_SUB_PRIOR_0));

//Pin Config
AD1PCFG = 0xFFFE;        //set all AN pins to digital except AN0
        
//Set Pin Directions
TRISB = 0b1000000000100001;       //AREF, VBUSON, USB_Fault set to input
TRISCbits.TRISC13 = 0;              
TRISCbits.TRISC14 = 0;
TRISD = 0b1000011000100001;       //BUT, SDA, SCL set to input
TRISE = 0x0000;
TRISFbits.TRISF1 = 0;
//TRISF = 0x0000;
//

// Set pin Values
LATB = 0x0000;
LATCbits.LATC13 = 0;
LATCbits.LATC14 = 0;  
LATD = 0x0000;
LATE = 0x0000;
LATFbits.LATF1 = 0; 

}

/*********************************************************************
 * Function:        void Func_ShowImAlive(void)
 * PreCondition:
 * Input:           None
 * Output:          None
 * Side Effects:
 * Overview:     Used to blink LED2 to indicate running state
 * Note:
 ********************************************************************/
void Func_ShowImAlive(){
    int iTempCount;
    for(iTempCount = 0; iTempCount <= 5; iTempCount++){
        LATDbits.LATD1 = 1; 
        delay_ms (500);
        LATDbits.LATD1 = 0; 
        delay_ms (500);
    }
    
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
    
//Initialize the board
//Func_InitializeBoard();
    
   //Setup interrupts
 //removed temporarily
 //ConfigINT1(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_ENABLE); // Config INT1              //Acellerometer 1 : Data Ready

// ConfigINT4(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_ENABLE); // Config INT4              //Acellerometer 2 : Data Ready

 INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
 INTEnableInterrupts();

// Enable optimal performance
INTEnableSystemMultiVectoredInt();
SYSTEMConfigPerformance(GetSystemClock());
mOSCSetPBDIV(OSC_PB_DIV_1);            // Use 1:1 CPU Core:Peripheral clocks
OpenCoreTimer(CORE_TICK_RATE);         // Open 1 ms Timer


// set up the core timer interrupt with a priority of 2 and zero sub-priority
mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_2 | CT_INT_SUB_PRIOR_0));

//Pin Config
AD1PCFG = 0xFFFE;        //set all AN pins to digital except AN0
        
//Set Pin Directions
TRISB = 0b1000000000100001;       //AREF, VBUSON, USB_Fault set to input
TRISCbits.TRISC13 = 0;              
TRISCbits.TRISC14 = 0;
TRISD = 0b1000011000100001;       //BUT, SDA, SCL set to input
TRISE = 0x0000;
TRISFbits.TRISF1 = 0;
//TRISF = 0x0000;
//

// Set pin Values
LATB = 0x0000;
LATCbits.LATC13 = 0;
LATCbits.LATC14 = 0;  
LATD = 0x0000;
LATE = 0x0000;
LATFbits.LATF1 = 0; 

//Initialize the current time Struct and other timer vars
//struct TimeStamp TNow;
func_InitializeTime(&TNow);

//Scan the network for connected devices - if detected then allow trying to
// read from them.
//============================================================================//
iDeviceCount = ScanNetwork();

//Blink the LED on power-up
Func_ShowImAlive();





//printf((const char *)"Reading Values \r\n");
    //printf ((const char *)"iInputValue = %d \r\n", iInputValue);
if(0 < iDeviceCount)  //initializes the i2c netwrk and scans for devices
{
    //Now Initialise and calibrate each MMA8452
    for(iTempCount = 1; iTempCount <=2; iTempCount++)
    {
        initMMA8452Q(ucAddressArray[iTempCount]);
        MMA8652FC_Calibration(ucDataArray, ucAddressArray[iTempCount]);
    }

    //Initialize the timers
    //========================================================================//
   // TestCycleTimer.StartTime = Func_StartTimer(TestCycleTimer);
    TestCycleTimer.StartTime = TNow;                                            //Main cycle timer
    TestCycleTimer.Setpt.hr = 1;
    func_GetRemainingTime(&TestCycleTimer, &TNow);

    LogTimer1.StartTime = TNow;
    LogTimer1.Setpt.sec = 30;
    func_GetRemainingTime(&LogTimer1, &TNow);

    LogTimer2.StartTime = TNow;
    LogTimer2.Setpt.sec = 30;
    func_GetRemainingTime(&LogTimer2, &TNow);

    while(!TestCycleTimer.TimerComplete)
    {
        //Update the System Timer and the remaining times
        //====================================================================//
        if(TimeUpdated == 1)
        {
            TimeUpdated = 0;
            Func_UpdateSystemTime(&TNow, rtcYear, rtcMon, rtcMday, rtcHour, rtcMin, rtcSec, irtc_mSec);
            func_GetRemainingTime(&TestCycleTimer, &TNow);
            func_GetRemainingTime(&LogTimer1, &TNow);
            func_GetRemainingTime(&LogTimer2, &TNow);
        }

        //Check if data is ready on either accelerometer
        //====================================================================//
        if((DataReady1 == 1) || (DataReady2==1))
        {
             if(DataReady1==1)
             {
                 //Reset the interrupt flag
                 DataReady1 = 0;
     
                 if(drvI2CReadRegisters(OUT_X_MSB_REG, ucDataArray, 6, MMA8452Q_ADDR_1))                // Read data output registers 0x01-0x06
                 {
                     //printf((const char *)"\r\n\r\n Acc 1 data ready");

                     // 12-bit accelerometer data
                     X1out_12_bit = ((short) (ucDataArray[0]<<8 | ucDataArray[1])) >> 4;                // Compute 12-bit X-axis acceleration output value
                     Y1out_12_bit = ((short) (ucDataArray[2]<<8 | ucDataArray[3])) >> 4;                // Compute 12-bit Y-axis acceleration output value
                     Z1out_12_bit = ((short) (ucDataArray[4]<<8 | ucDataArray[5])) >> 4;                // Compute 12-bit Z-axis acceleration output value

                     // Accelerometer data converted to g's
                     X1out_g = ((float) X1out_12_bit) / SENSITIVITY_2G;                                 // Compute X-axis output value in g's
                     Y1out_g = ((float) Y1out_12_bit) / SENSITIVITY_2G;                                 // Compute Y-axis output value in g's
                     Z1out_g = ((float) Z1out_12_bit) / SENSITIVITY_2G;                                 // Compute Z-axis output value in g's

                     if(LogTimer1.TimerComplete)
                     {
                         //printf ((const char *)"\r\n\r\n Acc 1 x_out = %f, y_out = %f, z_out = %f ", X2out_g, Y2out_g, Z2out_g);
                         LogTimer1.StartTime = TNow;
                     }
                 }
             }

             if(DataReady2==1)
             {
                 DataReady2 = 0;

                 if(drvI2CReadRegisters(OUT_X_MSB_REG, ucDataArray, 6, MMA8452Q_ADDR_1))                // Read data output registers 0x01-0x06
                 {
                     //printf((const char *)"\r\n\r\n Acc 2 data ready");

                     // 12-bit accelerometer data
                     X2out_12_bit = ((short) (ucDataArray[0]<<8 | ucDataArray[1])) >> 4;                // Compute 12-bit X-axis acceleration output value
                     Y2out_12_bit = ((short) (ucDataArray[2]<<8 | ucDataArray[3])) >> 4;                // Compute 12-bit Y-axis acceleration output value
                     Z2out_12_bit = ((short) (ucDataArray[4]<<8 | ucDataArray[5])) >> 4;                // Compute 12-bit Z-axis acceleration output value

                         // Accelerometer data converted to g's
                     X2out_g = ((float) X2out_12_bit) / SENSITIVITY_2G;                                 // Compute X-axis output value in g's
                     Y2out_g = ((float) Y2out_12_bit) / SENSITIVITY_2G;                                 // Compute Y-axis output value in g's
                     Z2out_g = ((float) Z2out_12_bit) / SENSITIVITY_2G;                                 // Compute Z-axis output value in g's

                     if(LogTimer2.TimerComplete)
                     {
                         //printf ((const char *)"\r\n\r\n Acc 2 x_out = %f, y_out = %f, z_out = %f ", X2out_g, Y2out_g, Z2out_g);

                         LogTimer2.StartTime = TNow;
                     }
                 }
             }
        }
    }
}


//Alert user card being initilized
 //printf ((const char *)"Init SDCARD \r\n");
delay_ms (50);

//Initialize Disk
disk_initialize(0);

//Aert user SD card is being mounted
//printf ((const char *)"Mount SDCARD \r\n");
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

//printf ((const char *)"Writing Data \r\n");

//write data to data1.txt
for (iTempCount = 0; iTempCount < 1440; iTempCount++){
    //unsigned char buf[8];
    unsigned char buf[12];

    //convert values[] to ascii
    //itoa(buf, values[iTempCount], 10);


    //pointer to converted value
    //const char *text2 = buf;

    //convert values[] to ascii and pass back a pointer to the result string ---------------- BTA modified so test it!!!
    //const char *text2 = itoa(values[iTempCount]);
    My_itoa(values[iTempCount], buf, sizeof(buf));

    //write that value into text file
    //f_write(&file1, text2, strlen(text2), &len);
    f_write(&file1, buf, strlen(buf), &len);
 }


//Alert user file is closing
//printf ((const char *)"Close File \r\n");

//Close data.txt
f_close(&file1);

//Alert user card is unmounting
//printf ((const char *)"SD Card Unmounted \r\n");
//unmount filesystem
f_mount(0,NULL);

//delay some time
delay_ms(1000);

}


//Initialize the I2C Accel
//===========================================================================================
// Configure pin1 for wakeup.  Connect MMA8452Q INT2 pin to imp pin1.
//.pin1.configure(DIGITAL_IN_WAKEUP, pollMMA8452Q)

// set the I2C clock speed. We can do 10 kHz, 50 kHz, 100 kHz, or 400 kHz
// i2c.configure(CLOCK_SPEED_400_KHZ)
//i2c.configure(CLOCK_SPEED_100_KHZ) // try to fix i2c read errors.  May need 4.7K external pull-up to go to 400_KHZ
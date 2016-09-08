/*
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
#include "pic32mx\include\plib.h"
#include "pic32mx\include\stdlib.h"
#include "pic32mx\include\stdio.h"
#include "pic32mx\include\string.h"
#include "pic32mx\include\strings.h"

#include"GenericTypeDefs.h"
#include "MMA8452_Config.h"
#include "HardwareProfile.h"

#include "ff.h"
#include "GLOBAL_VARS.h"
#include "I2C_HardwareDrvr.h"

#include "DataLoggingDefs.h"

/*******************************************************************************
 Program variables
 ******************************************************************************/
 int iDeviceCount = 0;
 int iTempCount;

 BOOL bEndTest = FALSE;
 //int z = 0;         //count for lcd
 long values[1440]; //array for storing values
 
 
 UINT len;          //needed for writing file

 volatile UINT8  TimeUpdated; //Flags for accelerometer interrupts

 /******************************************************************************
* Sytem Timer - variables and constants
******************************************************************************/
volatile int irtc_mSec, mSec_CurrentCount = 0;
volatile BYTE rtcYear = 111, rtcMon = 11, rtcDay = 22;    // RTC date values
volatile BYTE rtcHour = 0, rtcMin = 0, rtcSec = 0;    // RTC time values
volatile unsigned long tick;                               // Used for ISR

 //Accell data variables
 SHORT X1out_12_bit, Y1out_12_bit, Z1out_12_bit, X2out_12_bit, Y2out_12_bit, Z2out_12_bit;
 float X1out_g, Y1out_g, Z1out_g, X2out_g, Y2out_g, Z2out_g;


//Work registers for fs command
DWORD acc_size;         /* Work register for fs command */
WORD acc_files, acc_dirs;
FILINFO Finfo;
const BYTE ft[] = {0,12,16,32};

//Data Logging Vars
volatile UINT8 ucSDActiveBuffer = 1;
volatile UINT8 ucSDDataBuffer_1[SDClusterSize];
volatile UINT16 usSDDataBuffer1Count = 0;
volatile UINT8 ucSDDataBuffer_2[SDClusterSize];
volatile UINT16 usSDDataBuffer2Count = 0;
UINT uiSDBytesWritten = 0;
BOOL bMoveBuffer1ToDisk = FALSE, 
     bMoveBuffer2ToDisk = FALSE;


//File system object
FATFS Fatfs;
FATFS *fs;            /* Pointer to file system object */
FIL file1, file2;      /* File objects */



/* Interrupt service routine of external int 1    */
void __ISR(_EXTERNAL_1_VECTOR,IPL2AUTO) INT1InterruptHandler(void){             //Acellerometer 1 : Data Ready
    
    if(0 == DataReady) {
           
        INTClearFlag(INT_INT1);                                                     // Clear the interrupt flag
        INTEnable(INT_I2C1B, INT_DISABLED);                                         // INT_I2C1M = I2C 1 Master Event; INT_I2C1B = I2C 1 Bus Collision; INT_INT1 = External Interrupt 1; INT_INT4 = External Interrupt 4; Event  INT_DISABLED; INT_ENABLED     
        INTEnable(INT_I2C1M, INT_DISABLED);                                         // INT_I2C1M = I2C 1 Master Event; INT_I2C1B = I2C 1 Bus Collision; INT_INT1 = External Interrupt 1; INT_INT4 = External Interrupt 4; Event  INT_DISABLED; INT_ENABLED

        DataReady = 1;                                                             //Global var to store which interrupt was last detected

        //Turn on the LED
        LATDbits.LATD1 = 1;                                                         //Visual indication of read status 

        //Kickoff the device read process --> Once this happens then the Master I2C interrupt handler should finish the read
        if(drvI2CReadRegisters(OUT_X_MSB_REG, ucDataArray, 6, MMA8452Q_ADDR_1)){  // Read data output registers 0x01-0x06
            DataReady = 0;
        }

        INTEnable(INT_I2C1B, INT_ENABLED);                                          // INT_I2C1M = I2C 1 Master Event; INT_I2C1B = I2C 1 Bus Collision; INT_INT1 = External Interrupt 1; INT_INT4 = External Interrupt 4; Event  INT_DISABLED; INT_ENABLED
        INTEnable(INT_I2C1M, INT_ENABLED);                                          // INT_I2C1M = I2C 1 Master Event; INT_I2C1B = I2C 1 Bus Collision; INT_INT1 = External Interrupt 1; INT_INT4 = External Interrupt 4; Event  INT_DISABLED; INT_ENABLED
    }
} 



/* Interrupt service routine of external int 4    */

void __ISR(_EXTERNAL_4_VECTOR,IPL2AUTO) INT4InterruptHandler(void)              //Acellerometer 2 : Data Ready
{ 
    if(0 == DataReady) {
       
        INTClearFlag(INT_INT4);                                                     // Clear the interrupt flag
        DataReady = 2;                                                             //Global var to store which interrupt was last detected
        INTEnable(INT_I2C1B, INT_DISABLED);                                         // INT_I2C1M = I2C 1 Master Event; INT_I2C1B = I2C 1 Bus Collision; INT_INT1 = External Interrupt 1; INT_INT4 = External Interrupt 4; Event  INT_DISABLED; INT_ENABLED
        INTEnable(INT_I2C1M, INT_DISABLED);                                         // INT_I2C1M = I2C 1 Master Event; INT_I2C1B = I2C 1 Bus Collision; INT_INT1 = External Interrupt 1; INT_INT4 = External Interrupt 4; Event  INT_DISABLED; INT_ENABLED

        //Turn on the LED
        LATDbits.LATD1 = 1;                                                         //Visual indication of read status

        //Kickoff the device read process --> Once this happens then the Master I2C interrupt handler should finish the read
        if(drvI2CReadRegisters(OUT_X_MSB_REG, ucDataArray, 6, MMA8452Q_ADDR_2)){ // Read data output registers 0x01-0x06
            DataReady = 0;
        }

        INTEnable(INT_I2C1B, INT_ENABLED);                                          // INT_I2C1M = I2C 1 Master Event; INT_I2C1B = I2C 1 Bus Collision; INT_INT1 = External Interrupt 1; INT_INT4 = External Interrupt 4; Event  INT_DISABLED; INT_ENABLED
        INTEnable(INT_I2C1M, INT_ENABLED);                                          // INT_I2C1M = I2C 1 Master Event; INT_I2C1B = I2C 1 Bus Collision; INT_INT1 = External Interrupt 1; INT_INT4 = External Interrupt 4; Event  INT_DISABLED; INT_ENABLED
    }
}



///////////////////////////////////////////////////////////////////
//
// Master I2C interrupt handler
// This handler is called when qualifying I2C Master events occur
// this means that as well as Slave events 
// Master and Bus Collision events will also trigger this handler.
//

/*
 Master mode operations that generate a master interrupt are:
?Start Condition ? 1 BRG time after falling edge of SDAx
?Repeated Start Sequence ? 1 BRG time after falling edge of SDAx
?Stop Condition ? 1 BRG time after the rising edge of SDAx
?Data transfer byte received ? Eighth falling edge of SCLx (after receiving eight bits of data from slave)
?During a Send ACK sequence ? Ninth falling edge of SCLx (after sending ACK or NACK to slave)
?Data transfer byte transmitted ? Ninth falling edge of SCLx (regardless of receiving ACK from slave)
?During a slave-detected Stop ? When slave sets the P bit (I2CxSTAT<4>)
 * 
Slave mode operations that generate a slave interrupt are:
?Detection of a valid device address (including general call) ? Ninth falling edge of SCLx
(after sending ACK to master. Address must match unless the STRICT bit = 1 (I2CxCON<11>) or the GCEN bit =1 (I2CxCON<7>)
?Reception of data ? Ninth falling edge of SCLx (after sending the ACK to master)
?Request to transmit data ? Ninth falling edge of SCLx (regardless of receiving an ACK from the master)
 * 
Bus Collision events that generate an interrupt are:
?During a Start sequence ? SDAx sampled before Start condition
?During a Start sequence ? SCLx = 0 before SDAx = 0
?During a Start sequence ? SDAx = 0 before BRG time out
?During a Repeated Start sequence ? If SDAx is sampled 0 when SCLx goes high
?During a Repeated Start sequence ? If SCLx goes low before SDAx goes low
?During a Stop sequence ? If SDAx is sampled low after allowing it to float
?During a Stop sequence ? If SCLx goes low before SDAx goes high
 */

///////////////////////////////////////////////////////////////////
void __ISR(_I2C_1_VECTOR, IPL3AUTO) _MasterI2CHandler(void)
{
     // check for Slave and Bus events and return as we are not handling these
    if (IFS0bits.I2C1SIF == 1) {
        mI2C1SClearIntFlag(); 
    }
    else{
        
       // if(DataReady > 0){
            if (IFS0bits.I2C1MIF == 1) {
               mI2C1MClearIntFlag();                                                   // Clear the master interrupt flag if it is set
            }

            //May want to use this later to reset the current i2c operation
            if (IFS0bits.I2C1BIF == 1) {
                mI2C1BClearIntFlag();                                                   // Clear the bus fault interrupt flag if it is set
            }
        //}
        
        switch (DataReady)
        {

            case 1 : 

                /***************************************************************************
                 *                              Accel #1 read logic
                 * ************************************************************************/

                //Finish the read process that is currently running
                if(drvI2CReadRegisters(OUT_X_MSB_REG, ucDataArray, 6, MMA8452Q_ADDR_1))                // Read data output registers 0x01-0x06
                {
                    //Turn off the LED
                    LATDbits.LATD1 = 0;                                                 //Clear visual indication to signal that read is complete

                    DataReady = 0;                                                     // Reset var status
           
                    //Move sample to buffer 1
                    if(ucSDActiveBuffer == 1){
                        
                        if(usSDDataBuffer1Count < (SDClusterSize - SampleSize)){
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = 2;                   // Sensor #
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // x1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // x2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // y1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // y2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // z1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // z2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;             // Year
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Month
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Day
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;             // Hour
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Minute
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Second
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;    // Mili1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // Mili2               
                            //usSDDataBuffer1Count++;
                        }
                        else{
                            bMoveBuffer1ToDisk = TRUE;
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = 2;                   // Sensor #
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // x1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // x2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // y1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // y2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // z1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // z2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;             // Year
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Month
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Day
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;             // Hour
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Minute
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Second
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;    // Mili1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // Mili2
                        }
                    }
                    //Move sample to buffer 2
                    else {
                        if(usSDDataBuffer2Count < (SDClusterSize - SampleSize)){
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = 2;                   // Sensor #
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // x1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // x2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // y1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // y2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // z1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // z2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;             // Year
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Month
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Day
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;             // Hour
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Minute
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Second
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;    // Mili1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // Mili2
                            //usSDDataBuffer2Count++;
                        }
                        else{
                            bMoveBuffer2ToDisk = TRUE;
                            
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = 2;                   // Sensor #
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // x1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // x2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // y1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // y2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // z1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // z2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;             // Year
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Month
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Day
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;             // Hour
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Minute
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Second
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;    // Mili1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // Mili2
                        }
                    }
                    
                    /*
                    //Move sample to buffer 1
                    if(ucSDActiveBuffer == 1){
                        
                        if(usSDDataBuffer1Count < (SDClusterSize - SampleSize)){
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = 1;                   // Sensor #
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[0];      // x1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[1];      // x2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[2];      // y1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[3];      // y2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[4];      // z1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[5];      // z2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcYear;             // Year
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcMon;              // Month
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcDay;              // Day
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcHour;             // Hour
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcMin;              // Minute
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcSec;              // Second
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = irtc_mSec & 0xFF;    // Mili1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = irtc_mSec >> 8;      // Mili2               
                            //usSDDataBuffer1Count++;
                        }
                        else{
                            bMoveBuffer1ToDisk = TRUE;
                            ucSDDataBuffer_2[usSDDataBuffer1Count++] = 1;                   // Sensor #
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[0];      // x1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[1];      // x2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[2];      // y1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[3];      // y2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[4];      // z1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[5];      // z2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcYear;             // Year
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcMon;              // Month
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcDay;              // Day
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcHour;             // Hour
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcMin;              // Minute
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcSec;              // Second
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = irtc_mSec & 0xFF;    // Mili1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = irtc_mSec >> 8;      // Mili2
                        }
                    }
                    //Move sample to buffer 2
                    else {
                        if(usSDDataBuffer2Count < (SDClusterSize - SampleSize)){
                            ucSDDataBuffer_2[usSDDataBuffer1Count++] = 1;                   // Sensor #
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[0];      // x1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[1];      // x2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[2];      // y1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[3];      // y2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[4];      // z1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[5];      // z2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcYear;             // Year
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcMon;              // Month
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcDay;              // Day
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcHour;             // Hour
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcMin;              // Minute
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcSec;              // Second
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = irtc_mSec & 0xFF;    // Mili1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = irtc_mSec >> 8;      // Mili2
                            //usSDDataBuffer2Count++;
                        }
                        else{
                            bMoveBuffer2ToDisk = TRUE;
                            
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = 1;                   // Sensor #
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[0];      // x1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[1];      // x2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[2];      // y1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[3];      // y2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[4];      // z1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[5];      // z2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcYear;             // Year
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcMon;              // Month
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcDay;              // Day
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcHour;             // Hour
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcMin;              // Minute
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcSec;              // Second
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = irtc_mSec & 0xFF;    // Mili1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = irtc_mSec >> 8;      // Mili2
                        }
                    }
                        
                     */   
                        
      

                    // 12-bit accelerometer data
                  //  X1out_12_bit = ((short) (ucDataArray[0]<<8 | ucDataArray[1])) >> 4;                // Compute 12-bit X-axis acceleration output value
                   // Y1out_12_bit = ((short) (ucDataArray[2]<<8 | ucDataArray[3])) >> 4;                // Compute 12-bit Y-axis acceleration output value
                   // Z1out_12_bit = ((short) (ucDataArray[4]<<8 | ucDataArray[5])) >> 4;                // Compute 12-bit Z-axis acceleration output value

                    // Accelerometer data converted to g's
                  //  X1out_g = ((float) X1out_12_bit) / SENSITIVITY_2G;                                 // Compute X-axis output value in g's
                  //  Y1out_g = ((float) Y1out_12_bit) / SENSITIVITY_2G;                                 // Compute Y-axis output value in g's
                  //  Z1out_g = ((float) Z1out_12_bit) / SENSITIVITY_2G;                                 // Compute Z-axis output value in g's

                }

                break;
            case 2 :

                /***************************************************************************
                 *                              Accel #2 read logic
                 * ************************************************************************/

                //Finish the read process that is currently running
                if(drvI2CReadRegisters(OUT_X_MSB_REG, ucDataArray, 6, MMA8452Q_ADDR_2)) // Read data output registers 0x01-0x06
                {
                    //Turn off the LED
                    LATDbits.LATD1 = 0;                                                 //Clear visual indication to signal that read is complete

                    DataReady = 0;                                                     // Reset var status

                    if((PORTDbits.RD8 == 0x00) && (IFS0bits.INT1IF == 0)){
                        INTSetFlag(INT_INT1);
                    }
                    
                    //Move sample to buffer 1
                    if(ucSDActiveBuffer == 1){
                        
                        if(usSDDataBuffer1Count < (SDClusterSize - SampleSize)){
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = 2;                   // Sensor #
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // x1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // x2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // y1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // y2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // z1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // z2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;             // Year
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Month
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Day
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;             // Hour
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Minute
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Second
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;    // Mili1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // Mili2               
                            //usSDDataBuffer1Count++;
                        }
                        else{
                            bMoveBuffer1ToDisk = TRUE;
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = 2;                   // Sensor #
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // x1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // x2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // y1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // y2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // z1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // z2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;             // Year
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Month
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Day
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;             // Hour
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Minute
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Second
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;    // Mili1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // Mili2
                        }
                    }
                    //Move sample to buffer 2
                    else {
                        if(usSDDataBuffer2Count < (SDClusterSize - SampleSize)){
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = 2;                   // Sensor #
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // x1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // x2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // y1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // y2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // z1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // z2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;             // Year
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Month
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Day
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;             // Hour
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Minute
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;              // Second
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;    // Mili1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = usSDDataBuffer2Count;      // Mili2
                            //usSDDataBuffer2Count++;
                        }
                        else{
                            bMoveBuffer2ToDisk = TRUE;
                            
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = 2;                   // Sensor #
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // x1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // x2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // y1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // y2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // z1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // z2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;             // Year
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Month
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Day
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;             // Hour
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Minute
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;              // Second
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;    // Mili1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = usSDDataBuffer1Count;      // Mili2
                        }
                    }
                    /*
                    //Move sample to buffer 1
                    if(ucSDActiveBuffer == 1){
                        
                        if(usSDDataBuffer1Count < (SDClusterSize - SampleSize)){
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = 2;                   // Sensor #
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[0];      // x1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[1];      // x2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[2];      // y1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[3];      // y2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[4];      // z1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[5];      // z2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcYear;             // Year
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcMon;              // Month
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcDay;              // Day
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcHour;             // Hour
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcMin;              // Minute
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcSec;              // Second
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = irtc_mSec & 0xFF;    // Mili1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = irtc_mSec >> 8;      // Mili2               
                            //usSDDataBuffer1Count++;
                        }
                        else{
                            bMoveBuffer1ToDisk = TRUE;
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = 2;                   // Sensor #
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[0];      // x1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[1];      // x2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[2];      // y1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[3];      // y2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[4];      // z1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[5];      // z2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcYear;             // Year
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcMon;              // Month
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcDay;              // Day
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcHour;             // Hour
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcMin;              // Minute
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcSec;              // Second
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = irtc_mSec & 0xFF;    // Mili1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = irtc_mSec >> 8;      // Mili2
                        }
                    }
                    //Move sample to buffer 2
                    else {
                        if(usSDDataBuffer2Count < (SDClusterSize - SampleSize)){
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = 2;                   // Sensor #
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[0];      // x1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[1];      // x2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[2];      // y1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[3];      // y2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[4];      // z1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = ucDataArray[5];      // z2
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcYear;             // Year
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcMon;              // Month
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcDay;              // Day
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcHour;             // Hour
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcMin;              // Minute
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = rtcSec;              // Second
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = irtc_mSec & 0xFF;    // Mili1
                            ucSDDataBuffer_2[usSDDataBuffer2Count++] = irtc_mSec >> 8;      // Mili2
                            //usSDDataBuffer2Count++;
                        }
                        else{
                            bMoveBuffer2ToDisk = TRUE;
                            
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = 2;                   // Sensor #
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[0];      // x1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[1];      // x2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[2];      // y1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[3];      // y2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[4];      // z1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = ucDataArray[5];      // z2
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcYear;             // Year
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcMon;              // Month
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcDay;              // Day
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcHour;             // Hour
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcMin;              // Minute
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = rtcSec;              // Second
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = irtc_mSec & 0xFF;    // Mili1
                            ucSDDataBuffer_1[usSDDataBuffer1Count++] = irtc_mSec >> 8;      // Mili2
                        }
                    }
                     * */

                    // 12-bit accelerometer data
                  //  X1out_12_bit = ((short) (ucDataArray[0]<<8 | ucDataArray[1])) >> 4;                // Compute 12-bit X-axis acceleration output value
                   // Y1out_12_bit = ((short) (ucDataArray[2]<<8 | ucDataArray[3])) >> 4;                // Compute 12-bit Y-axis acceleration output value
                   // Z1out_12_bit = ((short) (ucDataArray[4]<<8 | ucDataArray[5])) >> 4;                // Compute 12-bit Z-axis acceleration output value

                    // Accelerometer data converted to g's
                  //  X1out_g = ((float) X1out_12_bit) / SENSITIVITY_2G;                                 // Compute X-axis output value in g's
                  //  Y1out_g = ((float) Y1out_12_bit) / SENSITIVITY_2G;                                 // Compute Y-axis output value in g's
                  //  Z1out_g = ((float) Z1out_12_bit) / SENSITIVITY_2G;                                 // Compute Z-axis output value in g's

                }

                break;

            default:
                break;
                
        }
    }
    
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
void __ISR(_CORE_TIMER_VECTOR, IPL1SOFT) CoreTimerHandler(void)
{
   
   static const BYTE dom[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

   BYTE n;

   // clear the interrupt flag
   mCTClearIntFlag();
    // update the period
    UpdateCoreTimer(CORE_TICK_RATE);

   disk_timerproc();   // call the low level disk IO timer functions
 //  tick++;            // increment the benchmarking timer

   if (++mSec_CurrentCount >= 86400030) //Allow up to 24 hours worth of timing
        mSec_CurrentCount = 0;
      
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
               if (++rtcDay > n) {
                  rtcDay = 1;
                  if (++rtcMon > 12) {
                     rtcMon = 1;
                     rtcYear++;
                  }
               }
            }
         }
      }
   }

   func_GetRemainingTime_ms(&msTestCycleTimer, mSec_CurrentCount);              // Update the test cycle timer
  
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

   IntStatus = INTDisableInterrupts();
   
   tmr =     (((DWORD)rtcYear - 80) << 25)
         | ((DWORD)rtcMon << 21)
         | ((DWORD)rtcDay << 16)
         | (WORD)(rtcHour << 11)
         | (WORD)(rtcMin << 5)
         | (WORD)(rtcSec >> 1);
   
    INTRestoreInterrupts(IntStatus);                                            // restore the interrupts to previous state
   return tmr;
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
        delay_ms (200);
        LATDbits.LATD1 = 0; 
        delay_ms (200);
    }
    
}


/*********************************************************************
 * Function:        void Func_ShowTestExit(void)
 * PreCondition:
 * Input:           None
 * Output:          None
 * Side Effects:
 * Overview:     Used to blink LED2 to indicate writing to sd card
 * Note:
 ********************************************************************/
void Func_ShowTestExit(){
    int iTempCount;
    for(iTempCount = 0; iTempCount <= 30; iTempCount++){
        LATDbits.LATD1 = 1; 
        delay_ms (100);
        LATDbits.LATD1 = 0; 
        delay_ms (100);
    }  
}

/*********************************************************************
 * Function:        void Func_DataLoggingInit(void)
 * PreCondition:
 * Input:           None
 * Output:          None
 * Side Effects:
 * Overview:     Used to initialize the disk and open the data file
 * Note:
 ********************************************************************/
void Func_DataLoggingInit(){
   
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
}

/*********************************************************************
 * Function:        void Func_DataLoggingCleanup(void)
 * PreCondition:
 * Input:           None
 * Output:          None
 * Side Effects:
 * Overview:     Used to close the data file and unmount the disk
 * Note:
 ********************************************************************/
void Func_DataLoggingCleanup(){
  
    //Close data.txt
    f_close(&file1);

    //unmount filesystem
    f_mount(0,NULL);

    //delay some time
    delay_ms(1000);
}

/*******************************************************************************
* Function: int main()
*
* Returns: Nothing
*
* Description: Program entry point, logs data for a 24 hour period then writes
*              it to a TEXT file n an SD card
*
*******************************************************************************/

int main (void)
{
 
/******************************************************************************/
//Initialize the board
/******************************************************************************/
    

INTEnableSystemMultiVectoredInt();
SYSTEMConfigPerformance(GetSystemClock());                                      // Enable optimal performance
mOSCSetPBDIV(OSC_PB_DIV_1);                                                     // Use 1:1 CPU Core:Peripheral clocks
OpenCoreTimer(CORE_TICK_RATE);                                                  // Open 1 ms Timer
INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
 

//Pin Config
AD1PCFG = 0xFFFF;                                                               //set all AN pins to digital except AN0
        
//Set Pin Directions
TRISB = 0b1000000000100001;       //AREF, VBUSON, USB_Fault set to input
TRISCbits.TRISC13 = 0;              
TRISCbits.TRISC14 = 0;

TRISDbits.TRISD0 = 0x1;
TRISDbits.TRISD1 = 0x0;
TRISDbits.TRISD2 = 0x0;
TRISDbits.TRISD3 = 0x0;
TRISDbits.TRISD4 = 0x0;
TRISDbits.TRISD5 = 0x0;
TRISDbits.TRISD6 = 0x0;
TRISDbits.TRISD7 = 0x0;
TRISDbits.TRISD8 = 0x1;
TRISDbits.TRISD9 = 0x1;    
TRISDbits.TRISD10 = 0x1;
TRISDbits.TRISD11 = 0x1;    
//TRISD = 0b0000111100100001;       //BUT, SDA, SCL set to input
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

/******************************************************************************/
//                          Configure the interrupts
/******************************************************************************/

// set up the core timer interrupt with a priority of 2 and zero sub-priority
mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_1 | CT_INT_SUB_PRIOR_0));

//Setup interrupts
ConfigINT1(EXT_INT_PRI_2 | FALLING_EDGE_INT | EXT_INT_ENABLE); // Config INT1              //Acellerometer 1 : Data Ready (External interrupt)
SetSubPriorityINT1(EXT_INT_SUB_PRI_2);

ConfigINT4(EXT_INT_PRI_2 | FALLING_EDGE_INT | EXT_INT_ENABLE); // Config INT4             //future
SetSubPriorityINT4(EXT_INT_SUB_PRI_2);

//==============================================================================
// Set up the I2C Master Event interrupt with priority level 
//==============================================================================
 // configure the interrupt priority for the I2C peripheral
 mI2C1SetIntPriority(I2C_INT_PRI_3);                                            //ISR priority level should match!!!!

INTEnable(INT_INT1, INT_DISABLED);                                   // INT_I2C1M = I2C 1 Master Event; INT_I2C1B = I2C 1 Bus Collision; INT_INT1 = External Interrupt 1; INT_INT4 = External Interrupt 4; Event  INT_DISABLED; INT_ENABLED
INTEnable(INT_INT4, INT_DISABLED);                                   // INT_I2C1M = I2C 1 Master Event; INT_I2C1B = I2C 1 Bus Collision; INT_INT1 = External Interrupt 1; INT_INT4 = External Interrupt 4; Event  INT_DISABLED; INT_ENABLED
                                                       
INTClearFlag(INT_INT4); 
INTClearFlag(INT_INT1); 

//Blink the LED on power-up
Func_ShowImAlive();

//Force the I2C slave to release the SDA line (sometimes the Slave will not release if the pic was reset during a transfer)
Func_ForceSlaveToReleaseSDA();

//Initialize the I2C network
drvI2CInit();

//Scan the network for connected devices - if detected then allow trying to
// read from them.
//============================================================================//
iDeviceCount = ScanNetwork(ucAddressArray);                         

if(0 < iDeviceCount)                                                            //initialize the i2c network 
{
    //Prepare the SD card, data file and buffers for use
    Func_DataLoggingInit();
    Func_FillTheBuffer(0,SDClusterSize,ucSDDataBuffer_1);
    Func_FillTheBuffer(0,SDClusterSize,ucSDDataBuffer_2);

    //Initialize the sensors
    MMA8452_Setup(iDeviceCount, NumInstalledAccels);
    
    INTEnable(INT_INT1, INT_ENABLED);                                   // INT_I2C1M = I2C 1 Master Event; INT_I2C1B = I2C 1 Bus Collision; INT_INT1 = External Interrupt 1; INT_INT4 = External Interrupt 4; Event  INT_DISABLED; INT_ENABLED
    INTEnable(INT_INT4, INT_ENABLED);                                   // INT_I2C1M = I2C 1 Master Event; INT_I2C1B = I2C 1 Bus Collision; INT_INT1 = External Interrupt 1; INT_INT4 = External Interrupt 4; Event  INT_DISABLED; INT_ENABLED
    
    //Timer for test run time
    msTestCycleTimer.StartTime = mSec_CurrentCount;                                            //Main cycle timer
    msTestCycleTimer.Setpt = 3600000; // 1 hour
    func_GetRemainingTime_ms(&msTestCycleTimer, mSec_CurrentCount);

    //Begin Main Loop    
    while(!msTestCycleTimer.TimerComplete && !bEndTest){
        
        if(PORTDbits.RD0 == 1){ //If Switch Pressed
            bEndTest = TRUE;
        }
        else{
            //Monitor to see if we need to dump the buffers to the SD card 
            if(bMoveBuffer1ToDisk){
                
                //Turn on LED2 to indicate writing to sd card
                LATDbits.LATD1 = 1;
                
                //write the data buffer to file
                f_write(&file1, ucSDDataBuffer_1, SDClusterSize, &uiSDBytesWritten);

                //Turn off LED2 to indicate writing to sd card has finished
                LATDbits.LATD1 = 0;
                
                //Clear out the buffer when finished
                Func_FillTheBuffer(0,SDClusterSize,ucSDDataBuffer_1);
                bMoveBuffer1ToDisk = FALSE;
            }
            else if(bMoveBuffer2ToDisk){
                
                //Turn on LED2 to indicate writing to sd card
                LATDbits.LATD1 = 1;
                
                //write the data buffer to file
                f_write(&file1, ucSDDataBuffer_2, SDClusterSize, &uiSDBytesWritten);

                //Turn off LED2 to indicate writing to sd card has finished
                LATDbits.LATD1 = 0;
                
                //Clear out the buffer when finished
                Func_FillTheBuffer(0,SDClusterSize,ucSDDataBuffer_2);
                bMoveBuffer2ToDisk = FALSE;
            }
        }
        
        
        
    }
   
    //Close out the file and unmount the SD card
    Func_DataLoggingCleanup();
    
    Func_ShowTestExit();
}

while(1);
//return 0;
}


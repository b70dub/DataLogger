/* 
 * File:   I2C_HardwareDrvr.h
 * Author: Brian
 *
 * Created on June 14, 2016, 8:40 AM
 */

#ifndef I2C_HARDWAREDRVR_H
#define	I2C_HARDWAREDRVR_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "pic32mx\include\xc.h"

 // Change below defines to I2C2xxx if using I2C2
#define I2CCON        I2C1CON
#define I2CCONbits    I2C1CONbits
#define I2CSTAT       I2C1STAT
#define I2CSTATbits   I2C1STATbits
#define I2CMSK        I2C1MSK
#define I2CRCV        I2C1RCV
#define I2CTRN        I2C1TRN
#define I2CADD        I2C1ADD
#define I2CBRG        I2C1BRG

#define FCL        800000// 400000 // Check the max speed supported by your peripheral!!

#define I2C_DEBUG   0 // Change to 1 for debug messages
 

/******************************************************************************
*I2C - functions
******************************************************************************/
UINT8 ScanNetwork(UINT8* ucAddressArray_ref);
BOOL get_ack_status(UINT8 address);

static BOOL I2C_Stop(void);
static BOOL I2C_SendByte(BYTE data);
void drvI2CInit(void);
BOOL drvI2CReadRegisters(UINT8 reg, volatile UINT8* rxPtr, UINT8 len, UINT8 slave_adr);//UINT8 slave_adr
BOOL drvI2CWriteRegisters(UINT8 reg, UINT8* data, UINT8 len, UINT8 slave_adr);
BOOL drvI2CWriteByte(UINT8 reg, UINT8 byte, UINT8 slave_adr);

//static BOOL I2C_Idle(void);
//static BOOL I2C_Start(void);

__inline__ static BOOL I2C_Idle(void) {                                                     //- Supporting Function
   //Check to see if the Master I2C state is inactive
    if(I2CCONbits.SEN || I2CCONbits.PEN || I2CCONbits.RCEN ||
            I2CCONbits.RSEN || I2CCONbits.ACKEN || I2CSTATbits.TRSTAT){
        return FALSE;
    } else{
        return TRUE;
    }   
}

__inline__ static BOOL I2C_Start(void){                                                    //- Supporting Function
    static UINT8 StartConditionStep = 1;
    
    BOOL bReturnValue = FALSE;
    
    switch (StartConditionStep){
        case 1 : // Step 1: wait for module idle, set the start condition
            
            if (I2C_Idle()) {
                // Enable the Start condition
                I2CCONbits.SEN = 1;
                StartConditionStep = 2;
            } 
            break;
            
        case 2 : //Step 2: Check for Bus collision and start condition
            
             // Check for collisions
            //If a bus collision occurred then clear the bus collision bit
            if(I2CSTATbits.BCL == 1){
                //Initiate a stop request (IF NOT PENDING)
                if(I2C_Stop()){
                    //Reset the start step
                    StartConditionStep = 1;
                } else {
                    I2CSTATbits.BCL = 1;
                }
            }
            else if(I2CSTATbits.S == 1) {
                if(I2C_Idle()){
                    StartConditionStep = 1;
                    bReturnValue = TRUE;
                }

            } else {                                                                //Unknown error occured
                //Initiate a stop request
                I2C_Stop();

                //Reset the start step
                StartConditionStep = 1;
            }
            
            break;
    }
    
    return bReturnValue;
    
}


#ifdef	__cplusplus
}
#endif

#endif	/* I2C_HARDWAREDRVR_H */


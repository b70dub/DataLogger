//-----------------------------------------------------------------------------
//
//  Generic i2c driver
//
//  Original Author: Govind Mukundan Previously only supported up to 100 hz via polling
//  Modified by: Brian Ankeny
//  -Modified heavily and changed to remove blocking code from read function. Is now state machine based
//  -Now supports up to 800 hz 
//  
//  ********************************* HISTORY ********************************
//
//  Version : 1.0
//  Date :
//  Description: Initial version
//
//
//-----------------------------------------------------------------------------


#include "GenericTypeDefs.h"
#include "I2C_HardwareDrvr.h"
#include "Delay_32.h"
#include "MMA8452_Config.h"

#if(I2C_DEBUG == 1)
#include "uart2.h"
#endif


#define GetSystemClock()        (80000000ul)
#define GetPeripheralClock()    (GetSystemClock() / (1 << OSCCONbits.PBDIV))



/* I2CBRG setting based on peripheral bus clock*/
/*
PBCLK       I2CxBRG     PGD(1)      Approximate FSCK (two rollovers of BRG)
50 MHz      0x037       104 ns              400 kHz
50 MHz      0x0F3       104 ns              100 kHz
40 MHz      0x02C       104 ns              400 kHz
40 MHz      0x0C2       104 ns              100 kHz
30 MHz      0x020       104 ns              400 kHz
30 MHz      0x091       104 ns              100 kHz
20 MHz      0x015       104 ns              400 kHz
20 MHz      0x060       104 ns              100 kHz
10 MHz      0x009       104 ns              400 kHz
10 MHz      0x02F       104 ns              100 kHz
Note 1: The typical value of the Pulse Gobbler Delay (PGD) is 104 ns. Refer 
*/

void drvI2CInit(void) {                                                         // Initialization Function
    
UINT16 temp = 0;
I2CCON = 0; // reset bits to 0
I2CCONbits.I2CEN = 0; // disable module
I2CCONbits.DISSLW = 0; // Slew Rate Control: set to 1 for any speed below 400 khz. Set to 0 for 400khz  //Added BTA
//I2CBRG = (GetPeripheralClock() / FCL) - (GetPeripheralClock() / 10000000) - 1; //Formula from datasheet

I2CBRG = 0x02C;  //set based on table above and FPBDIV (see hardwareprofile.h)
I2CSTAT = 0;
I2CCONbits.I2CSIDL = 1; // dont operate in idle mode
//I2CCONbits.SCLREL = 1;
I2CCONbits.I2CEN = 1; // enable module
temp = I2CRCV; // clear RBF flag
    
//==============================================================================
// Set up the I2C Master Event interrupt with priority level 
//==============================================================================

    
//INTClearFlag( INT_I2C1 ); 
//INTSetVectorPriority( INT_I2C_1_VECTOR, INT_PRIORITY_LEVEL_2); 
//INTSetVectorSubPriority( INT_I2C_1_VECTOR, INT_SUB_PRIORITY_LEVEL_1 ); 

 // configure the interrupt priority for the I2C peripheral
 mI2C1SetIntPriority(I2C_INT_PRI_3);                                            //ISR priority level should match!!!!
 
 // clear pending interrupts and enable I2C interrupts for the master
 mI2C1MClearIntFlag();
 EnableIntMI2C1;                                                                //enables the master i2c interrupt  ---> DisableIntMI2C1 // for disable interrupt
  
    
#if(I2C_DEBUG == 1)
    UART2PrintString("Configured i2c1\n");
#endif

}


// This function writes the slave address to the i2c bus.
// If a slave chip is at that address, it should respond to
// this with an "ACK".   This function returns TRUE if an
// ACK was found.  Otherwise it returns FALSE.
BOOL get_ack_status(UINT8 address){
    
    UINT8 status;
    int iTempCount, iAckReceived = 0, iTestbit;

    for (iTempCount = 0; iTempCount < 100; iTempCount++) // wait for ACK for some time
    {
        //1. i2c start
        while(!I2C_Start()){}
        
        
        //2. Set Slave in W Mode
        while(!I2C_SendByte((address << 1) | 0)){} //Was | 0
            
        //3. Check ACK
        while(!I2C_Idle()){}
  
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            iAckReceived = 1;
            break;
        }
    }

    //status = I2C_SendByte(address);  // Status = 0 if got an ACK
    //4. Send the Stop 
    while(!I2C_Stop()){}

    if(iAckReceived == 1)
       return(TRUE);
    else 
       return(FALSE);
}

UINT8 ScanNetwork(UINT8* ucAddressArray_ref) {
    
    UINT8 address;
    UINT8 status;
    UINT8 count = 0;

    int iTestbit = 0;

    
    //printf((const char *)"\n\rStart:\n\r");

    //delay_ms(1000);

    // Try all slave addresses from 0x10 to 0xEF.
    // See if we get a response from any slaves
    // that may be on the i2c bus.
    for(address=0x10; address < 0xF0; address+=1){
        if(address == 0x1A)
            iTestbit = 1;

        if(address == 0x1D)
            iTestbit = 1;

        if(get_ack_status(address))
          {
           //printf((const char *)"ACK addr: %X\n\r", address);
           count++;
           ucAddressArray_ref[count] = address;
         //  delay_ms(2000);
          }
    }
    
    if(count == 0)
        iTestbit = 0;
       //printf((const char *)"\n\rNothing Found");
    else
        iTestbit = 1;
       //printf((const char *)"\n\rNumber of i2c chips found: %u", count);

    return count;
}

static BOOL I2C_Idle(void) {                                                     //- Supporting Function
   static UINT8 t = 10;
    /* Wait until I2C Bus is Inactive */
   
    if (I2CCONbits.SEN || I2CCONbits.PEN || I2CCONbits.RCEN ||
             I2CCONbits.RSEN || I2CCONbits.ACKEN || I2CSTATbits.TRSTAT){
        //t = 10;
        return FALSE;
    }
 //   if (t>0){

  //      t--;
  //      return FALSE;
 //   }
    else
    {
        return TRUE;
    }
    
}

static BOOL I2C_Start(void) {                                                    //- Supporting Function
    static UINT8  StartConditionStep = 1;
   
    // Step 1: wait for module idle, set the start condition and check for bus collision
    if(StartConditionStep == 1){
        if(I2C_Idle()){
            // Enable the Start condition
            I2CCONbits.SEN = 1;

            // Check for collisions
            if (I2CSTATbits.BCL) {
                StartConditionStep = 1;
                I2CSTATbits.BCL = 0;
                return FALSE;
            } else {
                StartConditionStep = 2;
            }
        } else {
            return FALSE;
        }
    } 
   
    //Step 2: Wait for bus idle and reset the step number
    if (StartConditionStep == 2){
        
        // wait for module idle
        if(I2C_Idle()){
            StartConditionStep = 1;
            return TRUE;
        } else {
            return FALSE;
        }
    }
}

static BOOL I2C_Stop(void) {                                                     //- Supporting Function
    static UINT8  StopConditionStep = 1, StopDelayCount = 0;
    int x = 0;
    //Step 1: wait for module idle
    
    if(StopConditionStep == 1){
        if(I2C_Idle()){
            StopConditionStep = 2;
            
            //initiate stop bit
            I2CCONbits.PEN = 1;
        } else {
            return FALSE;
        }
        
    }
    
    //Step 2: Set the stop condition
    if(StopConditionStep == 2){
        //wait for hardware clear of stop bit
        if((!I2CCONbits.PEN) || (StopDelayCount++ > 50)){
            StopConditionStep = 3;
        } else {
            return FALSE;
        }
    }
    
    //Step 2: Clear the control and status bits, wait for bus idle
    if(StopConditionStep == 3){
    
        I2CCONbits.RCEN = 0;
        // IFS1bits.MI2C1IF = 0; // Clear Interrupt
        I2CSTATbits.IWCOL = 0;
        I2CSTATbits.BCL = 0;
        // wait for module idle
        
        if(I2C_Idle()){
            StopConditionStep = 1;
            StopDelayCount = 0;
            return TRUE;                                                        //Success
        } else {
            return FALSE;
        }
    }
}

static BOOL I2C_SendByte(BYTE data){                                            //- Supporting Function
    BOOL bError = FALSE;
    static UINT8 SendByteStepNo = 0;
    /*
     *  TBF: Transmit Buffer Full Status bit
        1 = Transmit in progress; I2CxTRN register is full (8-bits of data)
        0 = Transmit complete; I2CxTRN register is empty
     */
    //If the send just started then initialize the send status vars
    if(SendByteStepNo == 0){
        SendByteStepNo = 1;
    }
    
    switch (SendByteStepNo)
    {
    
        case 1 : //Step 1: Wait for the tx buffer empty and then load the byte to transmit and check for a collision

            if(!I2CSTATbits.TBF) {
                I2CTRN = data;

                 // Check for collisions
                /*
                 * IWCOL: Write Collision Detect bit
                            1 = An attempt to write the I2CxTRN register collided because the I2C module is busy.
                            This bit must be cleared in software.
                            0 = No collision
                 */
                if ((I2CSTATbits.IWCOL == 1)) {  
                    SendByteStepNo = 0;
                    bError = TRUE;
                    return (FALSE);
                } else {
                    SendByteStepNo = 2;

                    //Step 2: Send the Byte
                    /*
                    TRSTAT: Transmit Status bit
                       In I2C Master mode only; applicable to Master Transmit mode.
                       1 = Master transmit is in progress (8 bits + ACK)
                       0 = Master transmit is not in progress
                    */
                   if(!I2CSTATbits.TRSTAT){                                                // wait until write cycle is complete

                       /*
                        BCL: Master Bus Collision Detect bit
                           Cleared when the I2C module is disabled (ON = 0).
                           1 = A bus collision has been detected during a master operation
                           0 = No collision has been detected
                        */
                       if ((I2CSTATbits.BCL == 1)) {
                           SendByteStepNo = 0;
                           bError = TRUE;
                           return (FALSE);
                       } else  {
                           SendByteStepNo = 3;

                           //Step 3: Check for Bus Idle
                            //if(I2C_Idle()){
                                SendByteStepNo = 0;
                                return (TRUE);                                                      //Success
                            //} else {
                            //    return FALSE;
                            //}  
                       }
                   } else {
                       return (FALSE);
                   }
                }
            } else {
                return (FALSE);
            }

        case 2 : //Step 2: Send the Byte

            /*
             TRSTAT: Transmit Status bit
                In I2C Master mode only; applicable to Master Transmit mode.
                1 = Master transmit is in progress (8 bits + ACK)
                0 = Master transmit is not in progress
             */
            if(!I2CSTATbits.TRSTAT){                                                // wait until write cycle is complete

                /*
                 BCL: Master Bus Collision Detect bit
                    Cleared when the I2C module is disabled (ON = 0).
                    1 = A bus collision has been detected during a master operation
                    0 = No collision has been detected
                 */
                if ((I2CSTATbits.BCL == 1)) {
                    SendByteStepNo = 0;
                    bError = TRUE;
                    return (FALSE);
                } else  {
                    SendByteStepNo = 3;

                    //Step 3: Check for Bus Idle
                   // if(I2C_Idle()){
                        SendByteStepNo = 0;
                    //    return (TRUE);                                                      //Success
                   // } else {
                   //     return FALSE;
                   // }   
                }
            } else {
                return (FALSE);
            }

        case 3 : //Step 3: Check for Bus Idle

          //  if(I2C_Idle()){
                SendByteStepNo = 0;
                return (TRUE);                                                      //Success
           // } else {
           //     return FALSE;
           // } 
    }
   
}


/**
 * @brief Read data from an I2C slave
 *
 * This function can be used to read one or more sequential registers from a slave.
 * To read multiple registers, the slave must support multi-byte reads.
 *
 * @param reg The register to start reading from (UINT8)
 * @param rxPtr A pointer to where the read data should be stored (UINT8*)
 * @param len Number of bytes/registers to read
 * @param slave_adr The 7 bit address of the slave without the R/W bits set
 * @return Boolean indicating if operation completed successfully or not
 */
BOOL drvI2CReadRegisters(UINT8 reg, volatile UINT8* rxPtr, UINT8 len, UINT8 slave_adr, volatile struct I2C_ReadStatuses* ReadStatus) {     //- Primary Use Function
//for accel reads len = 6
    UINT8 ret, j;
    BOOL ReturnVal = FALSE;
    static UINT8 ByteCount, ByteReadStep, AddressChkStep, ReadModeStep;
    
   INTDisableInterrupts();                                                            //Disable while running

    //If the read just started then initialize the read status vars
    if(ReadStatus->StepNo == 0)
    {
        ReadStatus->StepNo = 1;
        ReadStatus->Successful = FALSE;
        ReadStatus->ReadTries = 0;
        
        AddressChkStep = 1;
        ReadModeStep = 1;
        ByteCount = 0;
        ByteReadStep = 1;
        
       // MMA8452Q_SetMode(slave_adr, STANDBY);
    }
        
    //Step 1: Wait for Device addr ack
    if(ReadStatus->StepNo == 1){
        if(ReadStatus->ReadTries < 100){
            
            switch (AddressChkStep)
            {
            case 1 : //Step 1: i2c start
                if(I2C_Start()){
                    AddressChkStep = 2;

                    if(I2C_SendByte((slave_adr << 1) | 0)){
                        AddressChkStep = 3;
                        
                        if(I2C_Idle()){
                            if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                                ReadStatus->StepNo = 2;
                                ReadStatus->ReadTries = 0;

                                break;
                            } else{
                                ReadStatus->ReadTries++;
                                INTEnableInterrupts();  
                                return FALSE;
                            }
                        } else {
                            INTEnableInterrupts(); 
                            return FALSE;
                        }
                        
                    } else {
                        INTEnableInterrupts(); 
                        return FALSE;
                    }

                } else {
                    INTEnableInterrupts(); 
                    return FALSE;
                }

            case 2 : //Step 2: Set Slave in W Mode and Send the Byte

                if(I2C_SendByte((slave_adr << 1) | 0)){
                    AddressChkStep = 3;

                    if(I2C_Idle()){

                        if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                            ReadStatus->StepNo = 2;
                            ReadStatus->ReadTries = 0;

                            break;
                        } else{
                            ReadStatus->ReadTries++;
                            INTEnableInterrupts(); 
                            return FALSE;
                        }
                    } else {
                        INTEnableInterrupts(); 
                        return FALSE;
                    }

                } else {
                    INTEnableInterrupts(); 
                    return FALSE;
                }

            case 3 : //Step 3: Check for Bus Idle

                if(I2C_Idle()){
                    if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                        ReadStatus->StepNo = 2;
                        ReadStatus->ReadTries = 0;

                        break;
                    } else{
                        ReadStatus->ReadTries++;
                        INTEnableInterrupts(); 
                        return FALSE;
                    }
                } else {
                    INTEnableInterrupts(); 
                    return FALSE;
                }
            }
        }
        else{
            ReadStatus->Error = TRUE;                                            // Did we time out? -> Error and return 
            ReadStatus->StepNo = 0;
            INTEnableInterrupts(); 
            return FALSE;
        }
    }
    
    // Step2: if slave ackd then put the regno on the bus
    if(ReadStatus->StepNo == 2){
        
        if(I2C_SendByte(reg)){
            
            if (I2CSTATbits.ACKSTAT != 0){                                      // Did we receive an ACK ?
                ReadStatus->Error = TRUE;                                            // Did we time out? -> Error and return 
                ReadStatus->StepNo = 0;
                INTEnableInterrupts(); 
                return FALSE;                                              // Exit if there was a problem  (ReadStatus.Successful = False)
            }
            else{
                ReadStatus->StepNo = 3;
            }
        } else {
            INTEnableInterrupts(); 
            return FALSE;
        }
        
    }
    
    // Step3: Now that the register addres is setup, we can ask the slave to enter read mode.
    if(ReadStatus->StepNo == 3){        
        
        if(ReadStatus->ReadTries < 100){
            
            switch (ReadModeStep)
            {
            case 1 : //Step 1: i2c start
                if(I2C_Start()){
                    ReadModeStep = 2;

                    if(I2C_SendByte((slave_adr << 1) | 1)){
                        ReadModeStep = 3;
                        
                        if(I2C_Idle()){

                            if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                                ReadStatus->StepNo = 4;
                                ReadStatus->ReadTries = 0;

                                break;
                            } else{
                                ReadStatus->ReadTries++;
                                INTEnableInterrupts(); 
                                return FALSE;
                            }
                        } else {
                            INTEnableInterrupts(); 
                            return FALSE;
                        }
                        
                    } else {
                        INTEnableInterrupts(); 
                        return FALSE;
                    }

                } else {
                    INTEnableInterrupts(); 
                    return FALSE;
                }

            case 2 : //Step 2: Set Slave in R Mode and Send the Address Byte

                if(I2C_SendByte((slave_adr << 1) | 0)){
                    ReadModeStep = 3;

                    if(I2C_Idle()){

                        if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                            ReadStatus->StepNo = 4;
                            ReadStatus->ReadTries = 0;

                            break;
                        } else{
                            ReadStatus->ReadTries++;
                            INTEnableInterrupts(); 
                            return FALSE;
                        }
                    } else {
                        INTEnableInterrupts(); 
                        return FALSE;
                    }

                } else {
                    INTEnableInterrupts(); 
                    return FALSE;
                }

            case 3 : //Step 3: Check for Bus Idle

               if(I2C_Idle()){

                    if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                        ReadStatus->StepNo = 4;
                        ReadStatus->ReadTries = 0;

                        break;
                    } else{
                        ReadStatus->ReadTries++;
                        INTEnableInterrupts(); 
                        return FALSE;
                    }
                } else {
                    INTEnableInterrupts(); 
                    return FALSE;
                }
            }
        }
        else{
            ReadStatus->Error = TRUE;
            ReadStatus->StepNo = 1;
            INTEnableInterrupts(); 
            return FALSE;
        } 
    }

    // Step 4: Read in the bytes
    if(ReadStatus->StepNo == 4){
        
        if(ByteCount < len){
            switch (ByteReadStep)
            {
            case 1 : //Step 1: //Wait for Bus idle
                if(I2C_Idle())                                                  //Wait for Bus idle
                {
                    I2CCONbits.RCEN = 1;                                        // enable master read
                    ByteReadStep = 2;
                                   
                    if(!I2CCONbits.RCEN){                                       // wait for byte to be received !(I2CSTATbits.RBF) --rcen automatically clears when recvd
                        ByteReadStep = 3;
                        
                        if(I2C_Idle()){                                                 //WAIT for bus idle again
                            I2CSTATbits.I2COV = 0;                                      //Reset bit incase data came before we extracted the last set of data
                            *(rxPtr + ByteCount) = I2CRCV;

                            if ((ByteCount + 1) == len) {

                                //9. Generate a NACK on last byte
                                I2CCONbits.ACKDT = 1; // send nack
                                I2CCONbits.ACKEN = 1;

                                ByteReadStep = 4;

                                //10. generate a stop
                                if(I2C_Stop()){
                                    ByteCount++;
                                }

                                INTEnableInterrupts(); 
                                return FALSE; 

                            } else {
                                I2CCONbits.ACKDT = 0; // send ACK for sequential reads
                                I2CCONbits.ACKEN = 1;

                                 ByteCount++;
                                 ByteReadStep = 1;
                            }

                            INTEnableInterrupts(); 
                            return FALSE;

                        } else {
                            INTEnableInterrupts(); 
                            return FALSE;
                        }
                    } else {
                        INTEnableInterrupts(); 
                        return FALSE;
                    }
                } else {
                    INTEnableInterrupts(); 
                    return FALSE;
                }
                

            case 2 : //Step 2: Wait for Data received register
                
                    if(!I2CCONbits.RCEN){                                       // wait for byte to be received !(I2CSTATbits.RBF) --rcen automatically clears when recvd
                        ByteReadStep = 3;
                        
                        if(I2C_Idle()){                                                 //WAIT for bus idle again
                            I2CSTATbits.I2COV = 0;                                      //Reset bit incase data came before we extracted the last set of data
                            *(rxPtr + ByteCount) = I2CRCV;

                            if ((ByteCount + 1) == len) {

                                //9. Generate a NACK on last byte
                                I2CCONbits.ACKDT = 1; // send nack
                                I2CCONbits.ACKEN = 1;

                                ByteReadStep = 4;

                                //10. generate a stop
                                if(I2C_Stop()){
                                    ByteCount++;
                                }

                                INTEnableInterrupts(); 
                                return FALSE;   
                                
                            } else {
                                I2CCONbits.ACKDT = 0; // send ACK for sequential reads
                                I2CCONbits.ACKEN = 1;

                                 ByteCount++;
                                 ByteReadStep = 1;
                            }

                            INTEnableInterrupts(); 
                            return FALSE;

                        } else {
                            INTEnableInterrupts(); 
                            return FALSE;
                        }
                        
                    } else {
                        INTEnableInterrupts(); 
                        return FALSE;
                    }
            case 3 : //Step 3: Check for Bus Idle and move recvd data to storage location

                if(I2C_Idle()){                                                 //WAIT for bus idle again
                    I2CSTATbits.I2COV = 0;                                      //Reset bit incase data came before we extracted the last set of data
                    *(rxPtr + ByteCount) = I2CRCV;

                    if ((ByteCount + 1) == len) {
                        
                        //9. Generate a NACK on last byte
                        I2CCONbits.ACKDT = 1; // send nack
                        I2CCONbits.ACKEN = 1;
                        
                        ByteReadStep = 4;
                        
                        //10. generate a stop
                        if(I2C_Stop()){
                            ByteCount++;
                        }

                        INTEnableInterrupts(); 
                        return FALSE; 
                        
                    } else {
                        I2CCONbits.ACKDT = 0; // send ACK for sequential reads
                        I2CCONbits.ACKEN = 1;
                        
                         ByteCount++;
                         ByteReadStep = 1;
                    }
                   
                    INTEnableInterrupts(); 
                    return FALSE;
                    
                } else {
                    INTEnableInterrupts(); 
                    return FALSE;
                }
               
            case 4 : //Step 4: Send stop bit
                
                //10. generate a stop
                if(I2C_Stop()){
                    ByteCount++;
                }
                  
                INTEnableInterrupts(); 
                return FALSE;                   
            }
        }
        
        ReadStatus->StepNo = 0;
        //MMA8452Q_SetMode(slave_adr, ACTIVE); 
        INTEnableInterrupts(); 
        return TRUE;                                                            //Success
                                                 
    }
}


/**
 * @brief Write data into an I2C slave
 *
 * This function can be used to write one or more sequential registers from a slave.
 * To write multiple registers, the slave must support multi-byte writes.
 *
 * @param adr The register to start writing to (UINT8)
 * @param data A pointer to where the data should be fetched from (UINT8*)
 * @param len Number of bytes/registers to write
 * @param slave_adr The 7 bit address of the slave without the R/W bits set
 *
 * @return Boolean indicating if operation completed successfully or not
 */
BOOL drvI2CWriteRegisters(UINT8 adr, UINT8* data, UINT8 len, UINT8 slave_adr_Copy) {     //- Primary Use Function
    UINT8 i, flag, j;
    flag = 0;
    for (i = 0; i < 100; i++) {
        //1. i2c start
        while(!I2C_Start()){}
        //2. Set  in W Mode
        while(!I2C_SendByte((slave_adr_Copy << 1) | 0)){}  //select device by address and set to write mode
        //3. Check ACK
        while(!I2C_Idle()){}
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            flag = 1;
            break;
        }
#if(I2C_DEBUG == 1)
        UART2PutChar('.');
#endif
    }

    if (!flag) return (FALSE); // Exit if there was a problem
    // 4.if write cmd was successful, put the adress on the bus
    while(!I2C_SendByte(adr)){}
    while(!I2C_Idle()){}
    for (j = 0; j < len; j++) {
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            while(!I2C_SendByte(*(data + j))){}
        } else {
#if(I2C_DEBUG == 1)
            UART2PrintString("Error NACK Rxed\n");
#endif
            return FALSE;
        }
    }
    while(!I2C_Stop()){}

    return TRUE;

}

/**
 * @brief A wrapper around drvI2CWriteRegisters() to write only a byte of data
 *
 * @param reg The register to start reading from (UINT8)
 * @param byte The byte to write
 * @param slave_adr The 7 bit address of the slave without the R/W bits set
 * @return Boolean indicating if operation completed successfully or not
 */
BOOL drvI2CWriteByte(UINT8 reg, UINT8 byte, UINT8 slave_adr) {                      //- Primary Use Function
    return ( drvI2CWriteRegisters(reg, &byte, 1, slave_adr));
}

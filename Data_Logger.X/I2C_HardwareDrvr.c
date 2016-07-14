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

#include "GLOBAL_VARS.h"
#include "GenericTypeDefs.h"
#include "I2C_HardwareDrvr.h"
#include "Delay_32.h"
#include "MMA8452_Config.h"

#if(I2C_DEBUG == 1)
#include "uart2.h"
#endif


#define GetSystemClock()        (80000000ul)
#define GetPeripheralClock()    (GetSystemClock() / (1 << OSCCONbits.PBDIV))
#define INTEnableIFInitComplete() (InitComplete == 1) ? INTEnableInterrupts() : 0
                                   

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

    struct I2C_DeviceStatuses TempStatus = {FALSE,FALSE,0,0,0,0,0,0,0,0,0,0};
    volatile struct I2C_DeviceStatuses* CurrentDeviceStatus;
    
    
    if(address == MMA8452Q_ADDR_1){
        CurrentDeviceStatus = &Accel1Status;
    }
    else if(address == MMA8452Q_ADDR_2){
       CurrentDeviceStatus = &Accel2Status;
    }
    else
        CurrentDeviceStatus = &TempStatus;
         
        
    for (iTempCount = 0; iTempCount < 100; iTempCount++) // wait for ACK for some time
    {
        //1. i2c start
        while(!I2C_Start(CurrentDeviceStatus)){}
        
        
        //2. Set Slave in W Mode
        while(!I2C_SendByte( ((address << 1) | 0), CurrentDeviceStatus)){} //Was | 0
            
        //3. Wait for transmit complete
        while(I2CSTATbits.TRSTAT == 1){}
        
        //4. Check ACK
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            iAckReceived = 1;
            break;
        }
    }

    //status = I2C_SendByte(address, CurrentDeviceStatus);  // Status = 0 if got an ACK
    //4. Send the Stop 
    I2C_Stop();
    
    while(I2CSTATbits.P != 1){}

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
/*
 Master mode operations that generate a master interrupt are:
?Start Condition ? 1 BRG time after falling edge of SDAx                                                    (I2CCONbits.SEN == 0; SEN is reset 1 BRG time after falling edge of SDAx)
?Repeated Start Sequence ? 1 BRG time after falling edge of SDAx                                            (I2CCONbits.RSEN == 0)
?Stop Condition ? 1 BRG time after the rising edge of SDAx                                                  (I2CCONbits.PEN == 0)
?Data transfer byte received ? Eighth falling edge of SCLx (after receiving eight bits of data from slave)  (I2CCONbits.RCEN ==0)
?During a Send ACK sequence ? Ninth falling edge of SCLx (after sending ACK or NACK to slave)               (I2CCONbits.ACKEN == 0)
?Data transfer byte transmitted ? Ninth falling edge of SCLx (regardless of receiving ACK from slave)       (I2CSTATbits.TRSTAT == 0)
?During a slave-detected Stop ? When slave sets the P bit (I2CxSTAT<4>)
 
 */
static BOOL I2C_Idle(void) {                                                     //- Supporting Function
   //Check to see if the Master I2C state is inactive
    if(I2CCONbits.SEN || I2CCONbits.PEN || I2CCONbits.RCEN ||
            I2CCONbits.RSEN || I2CCONbits.ACKEN || I2CSTATbits.TRSTAT){
        return FALSE;
    } else{
        return TRUE;
    }
    
    
}

static BOOL I2C_Start(volatile struct I2C_DeviceStatuses* CurrentStatus) {                                                    //- Supporting Function
  //  static UINT8  StartConditionStep = 1;
   
    //If the start just started then initialize the start status vars
    if(CurrentStatus->StartConditionStep == 0){
        CurrentStatus->StartConditionStep = 1;
    }
    
    // Step 1: wait for module idle, set the start condition and check for bus collision
    if(CurrentStatus->StartConditionStep == 1){
     
        //Check for bus in idle state
        if (I2C_Idle()) {
            // Enable the Start condition
            I2CCONbits.SEN = 1;
            CurrentStatus->StartConditionStep = 2;
            return FALSE;

        } else {
            return FALSE;
        }
    } 
    
    //Step 2: Check for Bus collision and start condition
    if (CurrentStatus->StartConditionStep == 2){
        
         // Check for collisions
        //If a bus collision occurred then clear the bus collision bit
        if(I2CSTATbits.BCL == 1){
            I2CSTATbits.BCL = 0;
            
            //Initiate a stop request
            I2C_Stop();
            
            //Reset the start step
            CurrentStatus->StartConditionStep = 1;
            return FALSE;
        }
        else if(I2CSTATbits.S == 1) {
            CurrentStatus->StartConditionStep = 1;
            return TRUE;
        } else {                                                                //Unknown error occured
            //Initiate a stop request
            I2C_Stop();
            
            //Reset the start step
            CurrentStatus->StartConditionStep = 1;
            return FALSE;
        }   
    }  
}

static BOOL I2C_Stop(void) {                                                     //- Supporting Function
    //initiate stop bit
    I2CCONbits.PEN = 1;
    return FALSE;     
}
    
/* Initial call to this function should be preceded by a call to I2C_Start()*/    
static BOOL I2C_SendByte(BYTE data, volatile struct I2C_DeviceStatuses* CurrentStatus){                                            //- Supporting Function
    BOOL bError = FALSE;
  //  static UINT8 SendByteStepNo = 0;
    /*
     *  TBF: Transmit Buffer Full Status bit
        1 = Transmit in progress; I2CxTRN register is full (8-bits of data)
        0 = Transmit complete; I2CxTRN register is empty
     */
    //If the send just started then initialize the send status vars
    if(CurrentStatus->SendByteStepNo == 0){
        CurrentStatus->SendByteStepNo = 1;
    }
    
    switch (CurrentStatus->SendByteStepNo)
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
                    CurrentStatus->SendByteStepNo = 0;
                    CurrentStatus->BusCollision = TRUE;
                    bError = TRUE;
                } else {
                    CurrentStatus->SendByteStepNo = 2;                          //Now wait for master interrupt for transmit complete
                }
                
                return FALSE; 
                
            } else {
                return (FALSE);                                                 //Should never be in this condition (buffer should never be full in case 1)
            }
        
        case 2 : //Step 2: Wait for the Byte transmitted master mode interrupt

            //Step 2: Send the Byte
            /*
            TRSTAT: Transmit Status bit
               In I2C Master mode only; applicable to Master Transmit mode.
               1 = Master transmit is in progress (8 bits + ACK)
               0 = Master transmit is not in progress
            */
           if(!I2CSTATbits.TRSTAT){                                             //may not need this check as the logic should not return here until the transmit is complete

               /*
                BCL: Master Bus Collision Detect bit
                   Cleared when the I2C module is disabled (ON = 0).
                   1 = A bus collision has been detected during a master operation
                   0 = No collision has been detected
                */
               if ((I2CSTATbits.BCL == 1)) {
                   CurrentStatus->SendByteStepNo = 0;
                   CurrentStatus->BusCollision = TRUE;
                   return (FALSE);
               } else  {
                 
                   //Step 3: Check for Bus Idle                                 //Bus should already be idle if the transmit is complete? (Single master only)
                    //if(I2C_Idle()){
                        CurrentStatus->SendByteStepNo = 0;
                        return (TRUE);                                                      //Success
                    //} else {
                    //    return FALSE;
                    //}  
               }
           } else {
               return (FALSE);
           }
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
BOOL drvI2CReadRegisters(UINT8 reg, volatile UINT8* rxPtr, UINT8 len, UINT8 slave_adr, volatile struct I2C_DeviceStatuses* CurrentStatus) {     //- Primary Use Function
//for accel reads len = 6
    UINT8 ret, j;
    BOOL ReturnVal = FALSE;
   // static UINT8 ByteCount, ByteReadStep, AddressChkStep, ReadModeStep;
   
    //if(InitComplete == 1){
    //    INTDisableInterrupts();                                                            //Disable while running
    //}

    //If the read just started then initialize the read status vars
    if(CurrentStatus->StepNo == 0)
    {
        CurrentStatus->StepNo = 1;
        CurrentStatus->Successful = FALSE;
        CurrentStatus->ReadTries = 0;
        CurrentStatus->AddressChkStep = 1;
        CurrentStatus->ReadModeStep = 1;
        CurrentStatus->ByteCount = 0;
        CurrentStatus->ByteReadStep = 1;
        
        CurrentStatus->BusCollision = FALSE;
        
       // MMA8452Q_SetMode(slave_adr, STANDBY);
    }
        
    //Step 1: Wait for Device addr ack
    if(CurrentStatus->StepNo == 1){
        if(CurrentStatus->ReadTries < 100){
            
            switch (CurrentStatus->AddressChkStep)
            {
            case 1 : //Step 1: i2c start
                if(I2C_Start(CurrentStatus)){
                    CurrentStatus->AddressChkStep = 2;
                    CurrentStatus->BusCollision = FALSE;
                       
                    if(I2C_SendByte(((slave_adr << 1) | 0), CurrentStatus)){
                        CurrentStatus->AddressChkStep = 3;
                        
                        // Wait for transmit complete
                        if(I2CSTATbits.TRSTAT == 0){
                            if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                                CurrentStatus->StepNo = 2;
                                CurrentStatus->ReadTries = 0;

                                break;
                            } else{
                                CurrentStatus->ReadTries++;
                                CurrentStatus->AddressChkStep = 1;
                                //INTEnableIFInitComplete();
                                return FALSE;
                            }
                        } else {
                            //INTEnableIFInitComplete(); 
                            return FALSE;
                        }
                        
                    } else {
                        //INTEnableIFInitComplete();
                        //Handle a bus collision event that happened in the send byte function
                        if(CurrentStatus->BusCollision == TRUE){
                            CurrentStatus->StepNo = 1;
                            
                            if(I2C_Start(CurrentStatus)){                       //Call the I2C_Start() to generate another interrupt
                                CurrentStatus->AddressChkStep = 2;
                                CurrentStatus->BusCollision = FALSE;
                            }
                        }
                        return FALSE;
                    }

                } else {
                    //INTEnableIFInitComplete(); 
                    return FALSE;
                }

            case 2 : //Step 2: Set Slave in W Mode and Send the Byte

                if(I2C_SendByte( ((slave_adr << 1) | 0), CurrentStatus)){
                    CurrentStatus->AddressChkStep = 3;

                    // Wait for transmit complete
                    if(I2CSTATbits.TRSTAT == 0){

                        if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                            CurrentStatus->StepNo = 2;
                            CurrentStatus->ReadTries = 0;

                            break;
                        } else{
                            CurrentStatus->ReadTries++;
                            CurrentStatus->AddressChkStep = 1;
                            //INTEnableIFInitComplete(); 
                            return FALSE;
                        }
                    } else {
                        //INTEnableIFInitComplete(); 
                        return FALSE;
                    }

                } else {
                    //INTEnableIFInitComplete(); 
                    
                    //Handle a bus collision event that happened in the send byte function
                    if(CurrentStatus->BusCollision == TRUE){
                        CurrentStatus->StepNo = 1;
                        if(I2C_Start(CurrentStatus)){                       //Call the I2C_Start() to generate another interrupt
                            CurrentStatus->AddressChkStep = 2;
                            CurrentStatus->BusCollision = FALSE;
                        }
                    }
                    
                    return FALSE;
                }

            case 3 : //Step 3: Check for Bus Idle
                   
                // Wait for transmit complete
                if(I2CSTATbits.TRSTAT == 0){
                    if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                        CurrentStatus->StepNo = 2;
                        CurrentStatus->ReadTries = 0;

                        break;
                    } else{
                        CurrentStatus->ReadTries++;
                        CurrentStatus->AddressChkStep = 1;
                        //INTEnableIFInitComplete(); 
                        return FALSE;
                    }
                } else {
                    //INTEnableIFInitComplete(); 
                    return FALSE;
                }
            }
        }
        else{
            CurrentStatus->Error = TRUE;                                            // Did we time out? -> Error and return 
            CurrentStatus->StepNo = 0;
            //INTEnableIFInitComplete(); 
            return FALSE;
        }
    }
    
    // Step2: if slave ackd then put the regno on the bus
    if(CurrentStatus->StepNo == 2){
        
        if(I2C_SendByte(reg, CurrentStatus)){
          
            if (I2CSTATbits.ACKSTAT != 0){                                      // Did we receive an ACK ?
                CurrentStatus->Error = TRUE;                                            // Did we time out? -> Error and return 
                CurrentStatus->StepNo = 0;
                //INTEnableIFInitComplete(); 
                return FALSE;                                              // Exit if there was a problem  (CurrentStatus.Successful = False)
            }
            else{
                CurrentStatus->StepNo = 3;
                CurrentStatus->ReadTries = 0;
            }
        } else {
            //INTEnableIFInitComplete(); 
            //Handle a bus collision event that happened in the send byte function
            if(CurrentStatus->BusCollision == TRUE){
                CurrentStatus->StepNo = 1;
                if(I2C_Start(CurrentStatus)){                       //Call the I2C_Start() to generate another interrupt
                    CurrentStatus->AddressChkStep = 2;
                    CurrentStatus->BusCollision = FALSE;
                }
            }
            return FALSE;
        }
        
    }
    
    // Step3: Now that the register addres is setup, we can ask the slave to enter read mode.
    if(CurrentStatus->StepNo == 3){        
        
        if(CurrentStatus->ReadTries < 100){
            
            switch (CurrentStatus->ReadModeStep)
            {
            case 1 : //Step 1: i2c start
                if(I2C_Start(CurrentStatus)){
                    CurrentStatus->BusCollision = FALSE;
                    CurrentStatus->ReadModeStep = 2;

                    if(I2C_SendByte( ((slave_adr << 1) | 1), CurrentStatus) ){
                        CurrentStatus->ReadModeStep = 3;
                        
                        // Wait for transmit complete
                        if(I2CSTATbits.TRSTAT == 0){

                            if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                                CurrentStatus->StepNo = 4;
                                CurrentStatus->ReadTries = 0;

                                break;
                            } else{
                                CurrentStatus->ReadTries++;
                                //INTEnableIFInitComplete(); 
                                return FALSE;
                            }
                        } else {
                            //INTEnableIFInitComplete(); 
                            return FALSE;
                        }
                        
                    } else {
                        //INTEnableIFInitComplete(); 
                        //Handle a bus collision event that happened in the send byte function
                        if(CurrentStatus->BusCollision == TRUE){
                            CurrentStatus->ReadModeStep = 1;
                            
                            if(I2C_Start(CurrentStatus)){                       //Call the I2C_Start() to generate another interrupt
                                CurrentStatus->ReadModeStep = 2;
                                CurrentStatus->BusCollision = FALSE;
                            }
                        }
                        return FALSE;
                    }

                } else {
                    //INTEnableIFInitComplete(); 
                    return FALSE;
                }

            case 2 : //Step 2: Set Slave in R Mode and Send the Address Byte

                if(I2C_SendByte( ((slave_adr << 1) | 0), CurrentStatus )){
                    CurrentStatus->ReadModeStep = 3;

                    // Wait for transmit complete
                    if(I2CSTATbits.TRSTAT == 0){

                        if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                            CurrentStatus->StepNo = 4;
                            CurrentStatus->ReadTries = 0;

                            break;
                        } else{
                            CurrentStatus->ReadTries++;
                            //INTEnableIFInitComplete(); 
                            return FALSE;
                        }
                    } else {
                        //INTEnableIFInitComplete(); 
                        return FALSE;
                    }

                } else {
                    //INTEnableIFInitComplete(); 
                    //Handle a bus collision event that happened in the send byte function
                    if(CurrentStatus->BusCollision == TRUE){
                        CurrentStatus->ReadModeStep = 1;
                        
                        if(I2C_Start(CurrentStatus)){                       //Call the I2C_Start() to generate another interrupt
                            CurrentStatus->ReadModeStep = 2;
                            CurrentStatus->BusCollision = FALSE;
                        }
                    }
                    return FALSE;
                }

            case 3 : //Step 3: Check for Bus Idle

                // Wait for transmit complete
               if(I2CSTATbits.TRSTAT == 0){

                    if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                        CurrentStatus->StepNo = 4;
                        CurrentStatus->ReadTries = 0;

                        break;
                    } else{
                        CurrentStatus->ReadTries++;
                        //INTEnableIFInitComplete(); 
                        return FALSE;
                    }
                } else {
                    //INTEnableIFInitComplete(); 
                    return FALSE;
                }
            }
        }
        else{
            CurrentStatus->Error = TRUE;
            CurrentStatus->StepNo = 1;
            //INTEnableIFInitComplete(); 
            return FALSE;
        } 
    }

    // Step 4: Read in the bytes
    if(CurrentStatus->StepNo == 4){
        
        if(CurrentStatus->ByteCount < len){
            switch (CurrentStatus->ByteReadStep)
            {
            case 1 : //Step 1: //Wait for Bus idle
                //if(I2C_Idle())                                                  //Wait for Bus idle
                //{
                    I2CCONbits.RCEN = 1;                                        // enable master read
                    CurrentStatus->ByteReadStep = 2;
                                   
                    if(!I2CCONbits.RCEN){                                       // wait for byte to be received !(I2CSTATbits.RBF) --rcen automatically clears when recvd
                        CurrentStatus->ByteReadStep = 3;
                        
                        //if(I2C_Idle()){                                                 //WAIT for bus idle again
                            I2CSTATbits.I2COV = 0;                                      //Reset bit incase data came before we extracted the last set of data
                            *(rxPtr + CurrentStatus->ByteCount) = I2CRCV;

                            if ((CurrentStatus->ByteCount + 1) == len) {

                                //9. Generate a NACK on last byte
                                I2CCONbits.ACKDT = 1; // send nack
                                I2CCONbits.ACKEN = 1;

                                CurrentStatus->ByteReadStep = 4;

                                //10. generate a stop
                                if (I2CSTATbits.P != 1){
                                    I2C_Stop();
                                } else {
                                    CurrentStatus->ByteCount++;
                                }

                                //INTEnableIFInitComplete(); 
                                return FALSE; 

                            } else {
                                I2CCONbits.ACKDT = 0; // send ACK for sequential reads
                                I2CCONbits.ACKEN = 1;

                                 CurrentStatus->ByteCount++;
                                 CurrentStatus->ByteReadStep = 1;
                            }

                            //INTEnableIFInitComplete(); 
                            return FALSE;

                        //} else {
                            //INTEnableIFInitComplete(); 
                        //    return FALSE;
                        //}
                    } else {
                        //INTEnableIFInitComplete(); 
                        return FALSE;
                    }
               // } else {
                    //INTEnableIFInitComplete(); 
                //    return FALSE;
               // }
                

            case 2 : //Step 2: Wait for Data received register
                
                    if(!I2CCONbits.RCEN){                                       // wait for byte to be received !(I2CSTATbits.RBF) --rcen automatically clears when recvd
                      //  CurrentStatus->ByteReadStep = 3;
                        
                        //if(I2C_Idle()){                                                 //WAIT for bus idle again
                            I2CSTATbits.I2COV = 0;                                      //Reset bit incase data came before we extracted the last set of data
                            *(rxPtr + CurrentStatus->ByteCount) = I2CRCV;

                            if ((CurrentStatus->ByteCount + 1) == len) {

                                //9. Generate a NACK on last byte
                                I2CCONbits.ACKDT = 1; // send nack
                                I2CCONbits.ACKEN = 1;

                                CurrentStatus->ByteReadStep = 3;

                                //10. generate a stop
                                if (I2CSTATbits.P != 1){
                                    I2C_Stop();
                                } else {
                                    CurrentStatus->ByteCount++;
                                }

                                //INTEnableIFInitComplete(); 
                                return FALSE;   
                                
                            } else {
                                I2CCONbits.ACKDT = 0; // send ACK for sequential reads
                                I2CCONbits.ACKEN = 1;

                                 CurrentStatus->ByteCount++;
                                 CurrentStatus->ByteReadStep = 1;
                            }

                            //INTEnableIFInitComplete(); 
                            return FALSE;

                       // } else {
                            //INTEnableIFInitComplete(); 
                        //    return FALSE;
                       // }
                        
                    } else {
                        //INTEnableIFInitComplete(); 
                        return FALSE;
                    }
                    /*
            case 3 : //Step 3: Check for Bus Idle and move recvd data to storage location

                if(I2C_Idle()){                                                 //WAIT for bus idle again
                    I2CSTATbits.I2COV = 0;                                      //Reset bit incase data came before we extracted the last set of data
                    *(rxPtr + CurrentStatus->ByteCount) = I2CRCV;

                    if ((CurrentStatus->ByteCount + 1) == len) {
                        
                        //9. Generate a NACK on last byte
                        I2CCONbits.ACKDT = 1; // send nack
                        I2CCONbits.ACKEN = 1;
                        
                        CurrentStatus->ByteReadStep = 4;
                        
                        //10. generate a stop
                        if (I2CSTATbits.P != 1){
                            I2C_Stop();
                        } else {
                            CurrentStatus->ByteCount++;
                        }

                        //INTEnableIFInitComplete(); 
                        return FALSE; 
                        
                    } else {
                        I2CCONbits.ACKDT = 0; // send ACK for sequential reads
                        I2CCONbits.ACKEN = 1;
                        
                         CurrentStatus->ByteCount++;
                         CurrentStatus->ByteReadStep = 1;
                    }
                   
                    //INTEnableIFInitComplete(); 
                    return FALSE;
                    
                } else {
                    //INTEnableIFInitComplete(); 
                    return FALSE;
                }
               */
            case 3 : //Step 4: Send stop bit
                
                //10. generate a stop (if not idle)
                if (I2CSTATbits.P != 1){
                    I2C_Stop();
                } else {
                    CurrentStatus->ByteCount++;
                }
                  
                //INTEnableIFInitComplete(); 
                return FALSE;                   
            }
        }
        
        CurrentStatus->StepNo = 0;
        //MMA8452Q_SetMode(slave_adr, ACTIVE); 
       // INTEnableIFInitComplete(); 
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
BOOL drvI2CWriteRegisters(UINT8 adr, UINT8* data, UINT8 len, UINT8 slave_adr_Copy, volatile struct I2C_DeviceStatuses* CurrentStatus) {     //- Primary Use Function
    UINT8 i, flag, j;
    flag = 0;
    for (i = 0; i < 100; i++) {
        //1. i2c start
        while(!I2C_Start(CurrentStatus)){}
        
        //2. Set  in W Mode
        while(!I2C_SendByte( ((slave_adr_Copy << 1) | 0), CurrentStatus )){
            //Redo the start if bus collision detected
            if(CurrentStatus->BusCollision == TRUE){
                CurrentStatus->StartConditionStep = 0;
                while(!I2C_Start(CurrentStatus)){ }   
            }
        
        }  //select device by address and set to write mode
        //3. Wait for transmit complete
        while(I2CSTATbits.TRSTAT == 1){}
        
        if (I2CSTATbits.ACKSTAT == 0){ // Did we receive an ACK ?
            flag = 1;
            break;
        }
#if(I2C_DEBUG == 1)
        UART2PutChar('.');
#endif
    }

    if (!flag) return (FALSE); // Exit if there was a problem
    
    // 4.if write cmd was successful, put the adress on the bus
    while(!I2C_SendByte(adr, CurrentStatus)){}
    
    //5. Wait for transmit complete
    while(I2CSTATbits.TRSTAT == 1){}
    for (j = 0; j < len; j++) {
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            while(!I2C_SendByte(*(data + j), CurrentStatus)){}
        } else {
#if(I2C_DEBUG == 1)
            UART2PrintString("Error NACK Rxed\n");
#endif
            return FALSE;
        }
    }
    
    I2C_Stop();
    
    while(I2CSTATbits.P != 1){}

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
BOOL drvI2CWriteByte(UINT8 reg, UINT8 byte, UINT8 slave_adr, volatile struct I2C_DeviceStatuses* CurrentStatus) {                      //- Primary Use Function
    return ( drvI2CWriteRegisters(reg, &byte, 1, slave_adr, CurrentStatus));
}

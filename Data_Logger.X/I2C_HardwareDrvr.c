//-----------------------------------------------------------------------------
//
//  Generic i2c driver
//
//  Author: Brian Ankeny
//  -state machine based
//  -supports up to 800 hz (max ODR of MMA8452)
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

    //Check for module stopped status
    while(!I2C_Stop()){}
         
    for (iTempCount = 0; iTempCount < 100; iTempCount++) // wait for ACK for some time
    {
        //Stop the module if needed
        while(!I2C_Stop()){}
        
        //1. i2c start
        while(!I2C_Start()){}
        
        //2. Set Slave in W Mode
        while(!I2C_SendByte( (address << 1) | 0)){} //Was | 0
            
        //3. Wait for transmit complete
        while(I2CSTATbits.TRSTAT == 1){}
        
        //4. Check ACK
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            iAckReceived = 1;
            break;
        }
    }

    //4. Send the Stop 
    while(!I2C_Stop()){}

    if(iAckReceived == 1)
       return(TRUE);
    else 
       return(FALSE);
}

void Func_ForceSlaveToReleaseSDA(void){
    UINT8 TempCount;
    TRISDbits.TRISD10 = 0x0;
    for (TempCount = 0; TempCount < 8; TempCount++) {
        LATDbits.LATD10 = 1;
        delay_us(3);
        LATDbits.LATD10 = 0;
        delay_us(3);
    }
    TRISDbits.TRISD10 = 0x1;

}
  
UINT8 ScanNetwork(UINT8* ucAddressArray_ref) {
    UINT16 temp = 0;
    UINT8 address;
    UINT8 count = 0;

    int iTestbit = 0;

    // Try all slave addresses from 0x10 to 0xEF.
    // See if we get a response from any slaves
    // that may be on the i2c bus.
    for(address=0x10; address < 0xF0; address+=1){
        if(address == 0x1A)
            iTestbit = 1;

        if(address == 0x1D)
            iTestbit = 1;

        if(get_ack_status(address)){
         count++;
         ucAddressArray_ref[count] = address;
        }
    }

    temp = I2CRCV; // clear RBF flag
    
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
/*
static BOOL I2C_Idle(void) {                                                     //- Supporting Function
   //Check to see if the Master I2C state is inactive
    if(I2CCONbits.SEN || I2CCONbits.PEN || I2CCONbits.RCEN ||
            I2CCONbits.RSEN || I2CCONbits.ACKEN || I2CSTATbits.TRSTAT){
        return FALSE;
    } else{
        return TRUE;
    }   
}
*/
/*
static BOOL I2C_Start(void) {                                                    //- Supporting Function
    static UINT8 StartConditionStep = 1;
    
    // Step 1: wait for module idle, set the start condition and check for bus collision
    if(StartConditionStep == 1){
     
        //Check for bus in idle state
        if (I2C_Idle()) {
            // Enable the Start condition
            I2CCONbits.SEN = 1;
            StartConditionStep = 2;
            return FALSE;

        } else {
            return FALSE;
        }
    } 
    
    //Step 2: Check for Bus collision and start condition
    if (StartConditionStep == 2){
        
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
            return FALSE;
        }
        else if(I2CSTATbits.S == 1) {
            if(I2C_Idle()){
                StartConditionStep = 1;
                return TRUE;
            }
            else{
                return FALSE;
            }
                
        } else {                                                                //Unknown error occured
            //Initiate a stop request
            I2C_Stop();
            
            //Reset the start step
            StartConditionStep = 1;
            return FALSE;
        }   
    }  
}
*/
static BOOL I2C_Stop(void) {                                                     //- Supporting Function
    if(I2CSTATbits.P != 1){
        //Check for bus collision
        if ((I2CSTATbits.BCL == 1)) {
            I2CSTATbits.BCL = 0;
        }
        
        //initiate stop bit
        if(I2CCONbits.PEN != 1){
            I2CCONbits.PEN = 1; 
        }
        return FALSE;         
    } else {
        return TRUE;
    }  
}
    
/* Initial call to this function should be preceded by a call to I2C_Start()*/    
static BOOL I2C_SendByte(BYTE data){                                            //- Supporting Function
    BOOL bError = FALSE;
    static UINT8 SendByteStepNo = 1;
    /*
     *  TBF: Transmit Buffer Full Status bit
        1 = Transmit in progress; I2CxTRN register is full (8-bits of data)
        0 = Transmit complete; I2CxTRN register is empty
     */
    
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
                    I2CSTATbits.IWCOL = 0;
                    SendByteStepNo = 1;
                    I2CBusCollision = TRUE;
                    bError = TRUE;
                } else {
                    SendByteStepNo = 2;                          //Now wait for master interrupt for transmit complete
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
                   SendByteStepNo = 1;
                   I2CBusCollision = TRUE;
                   return (FALSE);
               } else  {
                 
                   //Step 3: Check for Bus Idle                                 //Bus should already be idle if the transmit is complete? (Single master only)
                    SendByteStepNo = 1;
                    return (TRUE);                                                      //Success
               }
           } else {
               return (FALSE);
           }
    }
}


/*
 
 */
BOOL drvI2CReadRegisters(UINT8 reg, volatile UINT8* rxPtr, UINT8 len, UINT8 slave_adr) {     //- Primary Use Function
//for accel reads len = 6
    static UINT8 StepNo = 0;
    static UINT8 ReadTries = 0;
    static UINT8 AddressChkStep = 1;
    static UINT8 ReadModeStep = 1;
    static UINT8 ByteCount = 0;
    static UINT8 ByteReadStep = 0;
    
    //If the read just started then initialize the read status vars
    if(StepNo == 0)
    {
        StepNo = 1;
        ReadTries = 0;
        AddressChkStep = 1;
        ReadModeStep = 1;
        ByteCount = 0;
        ByteReadStep = 1;
        I2CBusCollision = FALSE;
    }
        
    //Step 1: Wait for Device addr ack
    if(StepNo == 1){
        if(ReadTries < 100){
            
            switch (AddressChkStep)
            {
            case 1 : //Step 1: i2c start
                if(I2C_Start()){
                    AddressChkStep = 2;
                    I2CBusCollision = FALSE;
                       
                    if(I2C_SendByte((slave_adr << 1) | 0)){
                        AddressChkStep = 3;
                        
                        // Wait for transmit complete
                        if(I2CSTATbits.TRSTAT == 0){
                            if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                                StepNo = 2;
                                ReadTries = 0;

                                break;
                            } else{
                                ReadTries++;
                                AddressChkStep = 1;
                                return FALSE;
                            }
                        } else {
                            return FALSE;
                        }
                        
                    } else {
                        //Handle a bus collision event that happened in the send byte function
                        if(I2CBusCollision == TRUE){
                            StepNo = 1;
                            
                            if(I2C_Start()){                       //Call the I2C_Start() to generate another interrupt
                                AddressChkStep = 2;
                                I2CBusCollision = FALSE;
                            }
                        }
                        return FALSE;
                    }

                } else {
                    return FALSE;
                }

            case 2 : //Step 2: Set Slave in W Mode and Send the Byte

                if(I2C_SendByte( (slave_adr << 1) | 0)){
                    AddressChkStep = 3;

                    // Wait for transmit complete
                    if(I2CSTATbits.TRSTAT == 0){

                        if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                            StepNo = 2;
                            ReadTries = 0;

                            break;
                        } else{
                            ReadTries++;
                            AddressChkStep = 1;
                            return FALSE;
                        }
                    } else {
                        return FALSE;
                    }

                } else {
                    //Handle a bus collision event that happened in the send byte function
                    if(I2CBusCollision == TRUE){
                        StepNo = 1;
                        if(I2C_Start()){                       //Call the I2C_Start() to generate another interrupt
                            AddressChkStep = 2;
                            I2CBusCollision = FALSE;
                        }
                    }
                    return FALSE;
                }

            case 3 : //Step 3: Wait for Transmit complete and check for slave ACK
                   
                // Wait for transmit complete
                if(I2CSTATbits.TRSTAT == 0){
                    if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                        StepNo = 2;
                        ReadTries = 0;

                        break;
                    } else{
                        ReadTries++;
                        AddressChkStep = 1;
                        return FALSE;
                    }
                } else {
                    return FALSE;
                }
            }
        }
        else{                                                                   // Did we time out? -> Error and return 
            StepNo = 0;
            return FALSE;
        }
    }
    
    // Step2: if slave ackd then put the register number on the bus
    if(StepNo == 2){
        
        if(I2C_SendByte(reg)){
          
            if (I2CSTATbits.ACKSTAT != 0){                                      // Did we receive an ACK ?  if not  -> Error and return   
                StepNo = 0;
                return FALSE;                                                   // Exit if there was a problem 
            }
            else{
                StepNo = 3;
                ReadTries = 0;
            }
        } else {
            //Handle a bus collision event that happened in the send byte function
            if(I2CBusCollision == TRUE){
                StepNo = 1;
                if(I2C_Start()){                                   //Call the I2C_Start() to generate another interrupt
                    AddressChkStep = 2;
                    I2CBusCollision = FALSE;
                }
            }
            return FALSE;
        }
    }
    
    // Step3: Now that the register address is setup, we can ask the slave to enter read mode.
    if(StepNo == 3){        
        
        if(ReadTries < 100){
            
            switch (ReadModeStep)
            {
            case 1 : //Step 1: i2c start
                if(I2C_Start()){
                    I2CBusCollision = FALSE;
                    ReadModeStep = 2;

                    if(I2C_SendByte( (slave_adr << 1) | 1) ){
                        ReadModeStep = 3;
                        
                        //Wait for Transmit complete and check for slave ACK
                        if(I2CSTATbits.TRSTAT == 0){

                            if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                                StepNo = 4;
                                ReadTries = 0;

                                break;
                            } else{
                                ReadTries++;
                                return FALSE;
                            }
                        } else {
                            return FALSE;
                        }
                    } else {
                        //Handle a bus collision event that happened in the send byte function
                        if(I2CBusCollision == TRUE){
                            ReadModeStep = 1;
                            
                            if(I2C_Start()){                       //Call the I2C_Start() to generate another interrupt
                                ReadModeStep = 2;
                                I2CBusCollision = FALSE;
                            }
                        }
                        return FALSE;
                    }
                } else { 
                    return FALSE;
                }

            case 2 : //Step 2: Set Slave in R Mode and Send the Address Byte

                if(I2C_SendByte( (slave_adr << 1) | 0 )){
                    ReadModeStep = 3;

                    //Wait for Transmit complete and check for slave ACK
                    if(I2CSTATbits.TRSTAT == 0){

                        if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                            StepNo = 4;
                            ReadTries = 0;

                            break;
                        } else{
                            ReadTries++;
                            return FALSE;
                        }
                    } else {
                        return FALSE;
                    }
                } else {
                    //Handle a bus collision event that happened in the send byte function
                    if(I2CBusCollision == TRUE){
                        ReadModeStep = 1;
                        
                        if(I2C_Start()){                       //Call the I2C_Start() to generate another interrupt
                            ReadModeStep = 2;
                            I2CBusCollision = FALSE;
                        }
                    }
                    return FALSE;
                }

            case 3 : //Step 3: Wait for Transmit complete and check for slave ACK

               if(I2CSTATbits.TRSTAT == 0){

                    if (I2CSTATbits.ACKSTAT == 0){                          // Did we receive an ACK ?
                        StepNo = 4;
                        ReadTries = 0;

                        break;
                    } else{
                        ReadTries++;
                        return FALSE;
                    }
                } else {
                    return FALSE;
                }
            }
        }
        else{
            StepNo = 1;
            return FALSE;
        } 
    }

    // Step 4: Read in the bytes
    if(StepNo == 4){
        
        if(ByteCount < len){
            switch (ByteReadStep)
            {
                
            case 1 : //Step 1: //Wait for Bus idle
                
                //Check for bus in idle state
                if (I2C_Idle()) {
                    I2CCONbits.RCEN = 1;                                        // enable master read
                    ByteReadStep = 2;

                    if(!I2CCONbits.RCEN){                                       // wait for byte to be received !(I2CSTATbits.RBF) --rcen automatically clears when recvd
                        ByteReadStep = 3;

                        I2CSTATbits.I2COV = 0;                                      //Reset bit incase data came before we extracted the last set of data
                        *(rxPtr + ByteCount) = I2CRCV;

                        if ((ByteCount + 1) == len) {

                            //9. Generate a NACK on last byte
                            I2CCONbits.ACKDT = 1; // send nack
                            I2CCONbits.ACKEN = 1;

                            ByteReadStep = 4;

                            //10. generate a stop
                            if(I2C_Stop()) {
                                ByteCount++;
                            } 
                            return FALSE; 

                        } else {
                            I2CCONbits.ACKDT = 0; // send ACK for sequential reads
                            I2CCONbits.ACKEN = 1;

                             ByteCount++;
                             ByteReadStep = 1;
                        }
                        return FALSE;
                    } else {
                        return FALSE;
                    }
                } else{
                    return FALSE;
                }
                
            case 2 : //Step 2: Wait for Data received register
                
                if(!I2CCONbits.RCEN){                                       // wait for byte to be received !(I2CSTATbits.RBF) --rcen automatically clears when recvd

                    I2CSTATbits.I2COV = 0;                                      //Reset bit incase data came before we extracted the last set of data
                    *(rxPtr + ByteCount) = I2CRCV;

                    if ((ByteCount + 1) == len) {

                        //9. Generate a NACK on last byte
                        I2CCONbits.ACKDT = 1; // send nack
                        I2CCONbits.ACKEN = 1;

                        ByteReadStep = 3;

                        //10. generate a stop
                        if(I2C_Stop()) {
                            StepNo = 0;
                            return TRUE;
                        }
                        return FALSE;   

                    } else {
                        I2CCONbits.ACKDT = 0; // send ACK for sequential reads
                        I2CCONbits.ACKEN = 1;

                         ByteCount++;
                         ByteReadStep = 1;
                    }
                    return FALSE;

                } else {
                    return FALSE;
                }

            case 3 : //Step 3: Send stop bit
                
                //10. generate a stop (if not idle)
                if(I2C_Stop()) {
                    StepNo = 0;
                    return TRUE;                                                //Success    
                }
                return FALSE;                   
            }
        }
        
        if(I2C_Stop()) {
            StepNo = 0;
            return TRUE;                                                            //Success      
        }
        else {
            return FALSE;
        }
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
        
        //Stop the module if needed
        while(!I2C_Stop()) {}
        
        //1. i2c start
        while(!I2C_Start()){}
        
        //2. Set  in W Mode
        while(!I2C_SendByte( (slave_adr_Copy << 1) | 0)){
            //Redo the start if bus collision detected
            if(I2CBusCollision == TRUE){
               
                //Stop the module if needed
                while(!I2C_Stop()) {}
                
                //Reissue the START
                while(!I2C_Start()){ } 
                I2CBusCollision = FALSE;
            }
        
        }  //select device by address and set to write mode
        //3. Wait for transmit complete
        while(I2CSTATbits.TRSTAT == 1){}
        
        if (I2CSTATbits.ACKSTAT == 0){ // Did we receive an ACK ?
            flag = 1;
            break;
        }
    }

    if (!flag) return (FALSE); // Exit if there was a problem
    
    // 4.if write cmd was successful, put the adress on the bus
    while(!I2C_SendByte(adr)){}
    
    //5. Wait for transmit complete
    while(I2CSTATbits.TRSTAT == 1){}
    for (j = 0; j < len; j++) {
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            while(!I2C_SendByte(*(data + j))){}
        } else {
            return FALSE;
        }
    }
    
    //Issue a stop
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

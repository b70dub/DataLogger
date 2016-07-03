//-----------------------------------------------------------------------------
//
//  Generic i2c driver
//
//  Author: Govind Mukundan
//
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

#if(I2C_DEBUG == 1)
#include "uart2.h"
#endif


#define GetSystemClock()        (80000000ul)
#define GetPeripheralClock()    (GetSystemClock() / (1 << OSCCONbits.PBDIV))


void drvI2CInit(void) {                                                         // Initialization Function
    UINT16 temp = 0;
    I2CCON = 0; // reset bits to 0
    I2CCONbits.I2CEN = 0; // disable module
    I2CBRG = (GetPeripheralClock() / FCL) - (GetPeripheralClock() / 10000000) - 1; //Formula from datasheet
    //I2CBRG = ((SSMGetCurrentPeripheralClock() / (2 * Fsck)) - 1);

    //I2Cfreq    = 400            ; I2C bus clock freqency, KHz !!! ERROR in formula above 400 KHz
    //I2Cbr    = (fcy/I2Cfreq-fcy/1000)/1000-1    ; Calculate I2CBRG reload value. 11111111 in DS is incorrect!

    I2CSTAT = 0;
    I2CCONbits.I2CSIDL = 1; // dont operate in idle mode
    //I2CCONbits.SCLREL = 1;
    I2CCONbits.I2CEN = 1; // enable module
    temp = I2CRCV; // clear RBF flag
#if(I2C_DEBUG == 1)
    UART2PrintString("Configured i2c1\n");
#endif

}


// This function writes the slave address to the i2c bus.
// If a slave chip is at that address, it should respond to
// this with an "ACK".   This function returns TRUE if an
// ACK was found.  Otherwise it returns FALSE.
UINT8 get_ack_status(UINT8 address)
{
UINT8 status;

status = I2C_SendByte(address);  // Status = 0 if got an ACK
I2C_Stop();

if(status == 0)
   return(TRUE);
else 
   return(FALSE);
}

UINT8 ScanNetwork(UINT8* ucAddressArray_ref) {
UINT8 address;
UINT8 status;
UINT8 count = 0;

int iTestbit = 0;

drvI2CInit();
//printf((const char *)"\n\rStart:\n\r");

delay_ms(1000);

// Try all slave addresses from 0x10 to 0xEF.
// See if we get a response from any slaves
// that may be on the i2c bus.
for(address=0x10; address < 0xF0; address+=2)
   {
    status = get_ack_status(address);
    if(status == TRUE)
      {
       //printf((const char *)"ACK addr: %X\n\r", address);
       count++;
       ucAddressArray_ref[count] = address;
       delay_ms(2000);
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

static void I2C_Idle(void) {                                                     //- Supporting Function
    UINT8 t = 255;
    /* Wait until I2C Bus is Inactive */
    while (I2CCONbits.SEN || I2CCONbits.PEN || I2CCONbits.RCEN ||
            I2CCONbits.RSEN || I2CCONbits.ACKEN || I2CSTATbits.TRSTAT || t--);
}

static BOOL I2C_Start(void) {                                                    //- Supporting Function
    // wait for module idle
    I2C_Idle();
    // Enable the Start condition
    I2CCONbits.SEN = 1;


    // Check for collisions
    if (I2CSTATbits.BCL) {
#if(I2C_DEBUG == 1)
        UART2PrintString("I2C Start Cond Error! \r\n");
        UART2PrintString("CON\n");
        UART2PutHexWord(I2CCON);

        UART2PrintString("STAT\n");
        UART2PutHexWord(I2CSTAT);
#endif
        return (FALSE);
    } else {
        // wait for module idle
        I2C_Idle();
        return (TRUE);
    }
}

static void I2C_Stop(void) {                                                     //- Supporting Function
    int x = 0;
    // wait for module idle
    I2C_Idle();
    //initiate stop bit
    I2CCONbits.PEN = 1;

    //wait for hardware clear of stop bit
    while (I2CCONbits.PEN) {
        if (x++ > 50) break;
    }
    I2CCONbits.RCEN = 0;
    // IFS1bits.MI2C1IF = 0; // Clear Interrupt
    I2CSTATbits.IWCOL = 0;
    I2CSTATbits.BCL = 0;
    // wait for module idle
    I2C_Idle();
}

static BOOL I2C_SendByte(BYTE data) {                                            //- Supporting Function
    //while(I2CSTATbits.TBF); //Wait till data is transmitted.
    // Send the byte
    I2CTRN = data;

    // Check for collisions
    if ((I2CSTATbits.IWCOL == 1)) {
#if(I2C_DEBUG == 1)
        UART2PrintString("IWOCL \n");
        UART2PrintString("CON\n");
        UART2PutHexWord(I2CCON);

        UART2PrintString("STAT\n");
        UART2PutHexWord(I2CSTAT);
#endif
        return (FALSE);
    } else {
        while (I2CSTATbits.TRSTAT); // wait until write cycle is complete
        if ((I2CSTATbits.BCL == 1)) {
#if(I2C_DEBUG == 1)
            UART2PrintString("I2C Tx Error!\n");
            UART2PrintString("CON\n");
            UART2PutHexWord(I2CCON);

            UART2PrintString("STAT\n");
            UART2PutHexWord(I2CSTAT);
#endif
            return (FALSE);
        } else {
            // wait for module idle
            I2C_Idle();
            return (TRUE);
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
BOOL drvI2CReadRegisters(UINT8 reg, UINT8* rxPtr, UINT8 len, UINT8 slave_adr) {     //- Primary Use Function

    UINT8 i, flag, ret, j;
    flag = 0;
    ret = FALSE;
#if(I2C_DEBUG == 1)
    UART2PrintString("\r\n-----Start read reg:-----\r\n");
    UART2PutHex(reg);
    UART2PrintString("\r\n");
#endif
    for (i = 0; i < 100; i++) // wait for ACK for some time
    {
        //1. i2c start
        I2C_Start();
        //2. Set Slave in W Mode
        I2C_SendByte((slave_adr << 1) | 0);
        //3. Check ACK
        I2C_Idle();
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            flag = 1;
            break;
        }
#if(I2C_DEBUG == 1)
        UART2PutChar('.');
#endif
    }

    if (!flag) return FALSE; // Exit if there was a problem
    flag = 0;
    // 4.if write cmd was successful, put the regno on the bus
    I2C_SendByte(reg);
    if (I2CSTATbits.ACKSTAT != 0) // Did we receive an ACK ?
    {
#if(I2C_DEBUG == 1)
        UART2PrintString("Error NACK Rxed\n");
#endif
        return FALSE; // Exit if there was a problem
    }
    // Now that the register addres is setup, we can ask the slave to enter read mode.
    for (j = 0; j < 100; j++) {
        //5.Issue a repeated start = a start cond without a prior stop
        I2C_Start();
        //6. Set Slave in R mode
        I2C_SendByte((slave_adr << 1) | 1);
        //7. Check ACK
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            flag = 1;
            break;
        }
#if(I2C_DEBUG == 1)
        UART2PutChar('.');
#endif
    }

    if (!flag) return FALSE; // Exit if there was a problem

    for (i = 0; i < len; i++) //read all the bytes
    {
        I2C_Idle();
        // got the ack, processor is in read mode
        //8. read the register
        I2CCONbits.RCEN = 1; // enable master read
        while (I2CCONbits.RCEN); // wait for byte to be received !(I2CSTATbits.RBF)

        I2C_Idle();
        I2CSTATbits.I2COV = 0;
        *(rxPtr + i) = I2CRCV;

        if ((i + 1) == len) {
            //9. Generate a NACK on last byte
            I2CCONbits.ACKDT = 1; // send nack
            I2CCONbits.ACKEN = 1;
            //10. generate a stop
            I2C_Stop();
        } else {
            I2CCONbits.ACKDT = 0; // send ACK for sequential reads
            I2CCONbits.ACKEN = 1;
        }

        ret = TRUE;
    }
#if(I2C_DEBUG == 1)
    for (i = 0; i < len; i++) {
        UART2PutChar('$');
        UART2PutHex(*(rxPtr + i));
    }
#endif

    return ret;
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
BOOL drvI2CWriteRegisters(UINT8 adr, UINT8* data, UINT8 len, UINT8 slave_adr) {     //- Primary Use Function
    UINT8 i, flag, j;
    flag = 0;
    for (i = 0; i < 100; i++) {
        //1. i2c start
        I2C_Start();
        //2. Set  in W Mode
        I2C_SendByte((slave_adr << 1) | 0);
        //3. Check ACK
        I2C_Idle();
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
    I2C_SendByte(adr);
    I2C_Idle();
    for (j = 0; j < len; j++) {
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            I2C_SendByte(*(data + j));
        } else {
#if(I2C_DEBUG == 1)
            UART2PrintString("Error NACK Rxed\n");
#endif
            return FALSE;
        }
    }
    I2C_Stop();

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

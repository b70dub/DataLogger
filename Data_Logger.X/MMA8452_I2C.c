
#include "MMA8452_Config.h"
#include "GenericTypeDefs.h"

// Wrappers for MMA8452Q.


void MMA8652FC_Calibration (UINT8* ucDataArray_ref, UINT8 slave_adr)
{
     char X_offset, Y_offset, Z_offset;

     UINT8* TempData;

     DataReady1 = 0;
     DataReady2 = 0;

     if(slave_adr == MMA8452Q_ADDR_1)
     {
         while (DataReady1 == 0){}                                              // Is a first set of data ready?  //this is for using an interrupt to determine if data is available
            DataReady1 = 0;
     }
     else if(slave_adr == MMA8452Q_ADDR_2)
     {
         while (DataReady2 == 0){}                                              // Is a second set of data ready?  //this is for using an interrupt to determine if data is available
            DataReady2 = 0;
     }



     MMA8452QSetMode(slave_adr, STANDBY);                                        // set to Standby mode

     drvI2CReadRegisters(OUT_X_MSB_REG, ucDataArray_ref, 6, slave_adr)          // Read data output registers 0x01-0x06

     Xout_12_bit = ((short) (ucDataArray_ref[0]<<8 | ucDataArray_ref[1])) >> 4; // Compute 12-bit X-axis acceleration output value
     Yout_12_bit = ((short) (ucDataArray_ref[2]<<8 | ucDataArray_ref[3])) >> 4; // Compute 12-bit Y-axis acceleration output value
     Zout_12_bit = ((short) (ucDataArray_ref[4]<<8 | ucDataArray_ref[5])) >> 4; // Compute 12-bit Z-axis acceleration output value

     X_offset = Xout_12_bit / 2 * (-1);                                         // Compute X-axis offset correction value
     Y_offset = Yout_12_bit / 2 * (-1);                                         // Compute Y-axis offset correction value
     Z_offset = (Zout_12_bit - SENSITIVITY_2G) / 2 * (-1);                      // Compute Z-axis offset correction value

     &TempData = X_offset
     drvI2CWriteRegisters(OFF_X_REG, TempData, 1, slave_adr);

     &TempData = Y_offset
     drvI2CWriteRegisters(OFF_Y_REG, TempData, 1, slave_adr);

     &TempData = Z_offset
     drvI2CWriteRegisters(OFF_Z_REG, TempData, 1, slave_adr);


     MMA8452QSetMode(slave_adr, ACTIVE);                                         // Active mode again
}

UINT8* readAccelData(UINT8* ucDataArray_ref, UINT8 slave_adr) {

    UINT8* TempData;
    int iTempCount, iLength = 6;

    MMA8452QSetMode(slave_adr, STANDBY);                                        // set to Standby mode
       
    if(drvI2CReadRegisters(OUT_X_MSB_REG, ucDataArray_ref, iLength, slave_adr)) // Read data output registers 0x01-0x06
    {
        for(iTempCount = 0; iTempCount <= iLength-1; iTempCount++)
        {
            ucDataArray_ref[iTempCount]
        }

        rawData = readSequentialRegs(OUT_X_MSB_REG, 3)  // Read the three raw data registers into data array
        foreach (i, val in rawData) {
            ucDataArray_ref[i] = math.floor(1000.0 * ((val < 128 ? val : val - 256) / ((64 >> maxG) + 0.0)))
                // HACK: in above calc maxG just happens to be (log2(full_scale) - 1)  see: const for FS_2G, FS_4G, FS_8G
            //convert to signed integer milliGs
        }
        return ucDataArray_ref
        
    }
    else return NULL;

    MMA8452QSetMode(slave_adr, ACTIVE);                                         // Active mode again
}

// Sets the MMA8452Q  mode.
// 0 == STANDBY for changing registers
// 1 == ACTIVE for outputting data
void MMA8452QSetMode(UINT8 slave_adr, int iMode) {
    UINT8* TempData;

    if(iMode == 0)
        &TempData = 0x00;                                                       // set to Standby mode
    else if(iMode == 1)
        &TempData = 0x39;                                                       // ODR = 1.56Hz, Active mode

    drvI2CWriteRegisters(CTRL_REG1, TempData, 1, slave_adr);
}

void initMMA8452Q(UINT8 slave_adr) {
// Initialize the MMA8452Q registers
// See the many application notes for more info on setting all of these registers:

    UINT8* TempData;

    *TempData = 0x40;                                                               // Reset all registers to POR values
    drvI2CWriteRegisters(CTRL_REG2, TempData, 1, slave_adr);

    delay_ms(1);          // ~1ms delay

    *TempData = 0x01;                                                               // +/-4g range with ~0.977mg/LSB  +/-2 g:0x00
    drvI2CWriteRegisters(XYZ_DATA_CFG_REG, TempData, 1, slave_adr);

    *TempData = 0x02;                                                               // High Resolution mode
    drvI2CWriteRegisters(CTRL_REG2, TempData, 1, slave_adr);

    *TempData = 0x00;                                                               // Push-pull, active low interrupt
    drvI2CWriteRegisters(CTRL_REG3, TempData, 1, slave_adr);

    *TempData = 0x01;                                                               // Enable DRDY interrupt
    drvI2CWriteRegisters(CTRL_REG4, TempData, 1, slave_adr);

    *TempData = 0x01;                                                               // DRDY interrupt routed to INT1 - PTA5
    drvI2CWriteRegisters(CTRL_REG5, TempData, 1, slave_adr);

    MMA8452QSetMode(slave_adr, ACTIVE);                                         // ODR = 1.56Hz, Active mode

/*
    I2C_WriteRegister(MMA8652FC_I2C_ADDRESS, CTRL_REG2, 0x40);          // Reset all registers to POR values

     delay_ms(1);          // ~1ms delay

     I2C_WriteRegister(MMA8652FC_I2C_ADDRESS, XYZ_DATA_CFG_REG, 0x00);   // +/-2g range with ~0.977mg/LSB
     I2C_WriteRegister(MMA8652FC_I2C_ADDRESS, CTRL_REG2, 0x02);          // High Resolution mode
     I2C_WriteRegister(MMA8652FC_I2C_ADDRESS, CTRL_REG3, 0x00);          // Push-pull, active low interrupt
     I2C_WriteRegister(MMA8652FC_I2C_ADDRESS, CTRL_REG4, 0x01);          // Enable DRDY interrupt
     I2C_WriteRegister(MMA8652FC_I2C_ADDRESS, CTRL_REG5, 0x01);          // DRDY interrupt routed to INT1 - PTA5
     I2C_WriteRegister(MMA8652FC_I2C_ADDRESS, CTRL_REG1, 0x39);          // ODR = 1.56Hz, Active mode
*/
    
} // initMMA8452Q

// now application specific functions


#include "MMA8452_Config.h"
#include "GLOBAL_VARS.h"
#include "proc/p32mx440f256h.h"

// Wrappers for MMA8452Q.

MMA8452_Setup(iDeviceCount, NumInstalledAccels);
void MMA8452_Setup(UINT8 NumDevicesDetected, UINT8 NumAccels){
    UINT8 iTempCount;
    //Now Initialise and calibrate each MMA8452
    for(iTempCount = 1; iTempCount <= NumDevicesDetected; iTempCount++)
    {
        if((ucAddressArray[iTempCount] == 0x1C) || (ucAddressArray[iTempCount] == 0x1D)){
        ucCurrentAddress = ucAddressArray[iTempCount];
        initMMA8452Q(ucCurrentAddress);                                         //initialize the accelerometers
        MMA8652FC_Calibration(ucDataArray, ucCurrentAddress);                   //calibrate the accelerometers
        }
        
        NumAccels--;
        if(NumAccels == 0){
            break;
        }
    }
}

void MMA8652FC_Calibration (UINT8* ucDataArray_ref, UINT8 slave_adr_Copy)
{
     char X_offset, Y_offset, Z_offset;

     UINT8 TempData1, TempCount, ucDataArray_Temp[6];
     UINT8* TempData = &TempData1;

     volatile struct I2C_DeviceStatuses* CurrentDeviceStatus;
     struct I2C_DeviceStatuses TempStatus = {FALSE,FALSE,0,0,0,0,0,0,0,0,0,0};
  
     if(slave_adr_Copy == MMA8452Q_ADDR_1){
         CurrentDeviceStatus = &Accel1Status;
     }
     else if(slave_adr_Copy == MMA8452Q_ADDR_2){
        CurrentDeviceStatus = &Accel2Status;
     }
     else{
         CurrentDeviceStatus = &TempStatus;
     }

     while(!drvI2CReadRegisters(OUT_X_MSB_REG, ucDataArray_ref, 6, slave_adr_Copy, CurrentDeviceStatus)){}          // Read data output registers 0x01-0x06

     MMA8452Q_SetMode(slave_adr_Copy, STANDBY);                                        // set to Standby mode
     
     for(TempCount = 0; TempCount <= 5; TempCount++){
         ucDataArray_Temp[TempCount] = (UINT8)(*(ucDataArray_ref + TempCount));
         TempData1 = 0;
     }
     
    
     Xout_12_bit = ((short) (ucDataArray_Temp[0]<<8 | ucDataArray_Temp[1])) >> 4; // Compute 12-bit X-axis acceleration output value
     Yout_12_bit = ((short) (ucDataArray_Temp[2]<<8 | ucDataArray_Temp[3])) >> 4; // Compute 12-bit Y-axis acceleration output value
     Zout_12_bit = ((short) (ucDataArray_Temp[4]<<8 | ucDataArray_Temp[5])) >> 4; // Compute 12-bit Z-axis acceleration output value

     X_offset = Xout_12_bit / 2 * (-1);                                         // Compute X-axis offset correction value
     Y_offset = Yout_12_bit / 2 * (-1);                                         // Compute Y-axis offset correction value
     Z_offset = (Zout_12_bit - SENSITIVITY_2G) / 2 * (-1);                      // Compute Z-axis offset correction value

     *TempData = X_offset;
     drvI2CWriteRegisters(OFF_X_REG, TempData, 1, slave_adr_Copy, CurrentDeviceStatus);

     *TempData = Y_offset;
     drvI2CWriteRegisters(OFF_Y_REG, TempData, 1, slave_adr_Copy, CurrentDeviceStatus);

     *TempData = Z_offset;
     drvI2CWriteRegisters(OFF_Z_REG, TempData, 1, slave_adr_Copy, CurrentDeviceStatus);


     MMA8452Q_SetMode(slave_adr_Copy, ACTIVE);                                         // Active mode again
}

// Sets the MMA8452Q  mode.
// 0 == STANDBY for changing registers
// 1 == ACTIVE for outputting data
void MMA8452Q_SetMode(UINT8 slave_adr, int iMode) {
    UINT8 TempData1;
    UINT8* TempData = &TempData1;

    struct I2C_DeviceStatuses* TempDeviceStatus;
    struct I2C_DeviceStatuses TempStatus;
    
    if(slave_adr == MMA8452Q_ADDR_1){
         TempDeviceStatus = &Accel1Status;
    }
    else if(slave_adr == MMA8452Q_ADDR_2){
         TempDeviceStatus = &Accel2Status;
    }
    else{
         TempDeviceStatus = &TempStatus;
    }
    
    if(iMode == 0) // set to Standby mode
        *TempData = 0x00;                   

    else if(iMode == 1) // set to Active mode
        *TempData = 0x01;     //  0x39: ODR = 1.56hz, 0x19 = 100hz, 0x11 = 200hz, 0x09 = 400Hz,Active mode, 0x01 = 800Hz,Active mode; 
            
    drvI2CWriteRegisters(CTRL_REG1, TempData, 1, slave_adr, TempDeviceStatus);
}

void initMMA8452Q(UINT8 slave_adr) {
// Initialize the MMA8452Q registers
// See the many application notes for more info on setting all of these registers:

    UINT8 TempData1, Temp_CTRLREG1, Temp_CTRLREG2 = 0, Temp_CTRLREG3 = 0, Temp_CTRLREG4, Temp_CTRLREG5;
    UINT8* TempData = &TempData1;

    volatile struct I2C_DeviceStatuses* TempDeviceStatus;
    struct I2C_DeviceStatuses TempStatus;
        
    if(slave_adr == MMA8452Q_ADDR_1){
         TempDeviceStatus = &Accel1Status;
    }
    else if(slave_adr == MMA8452Q_ADDR_2){
         TempDeviceStatus = &Accel2Status;
    }
    else{
         TempDeviceStatus = &TempStatus;
    }
         
   // Temp_CTRLREG1[0] = 0;
    //Temp_CTRLREG1[1] = 0;
    Temp_CTRLREG1 = 0;
    Temp_CTRLREG2 = 0;
    Temp_CTRLREG3 = 0;
    Temp_CTRLREG4 = 0;
    Temp_CTRLREG5 = 0;
    
 //   I2C_Start();
    MMA8452Q_SetMode(slave_adr, STANDBY); 
    
//    I2C_Start();
    while(!drvI2CReadRegisters(CTRL_REG1, &Temp_CTRLREG1, 1, slave_adr, TempDeviceStatus)) { }
    while(!drvI2CReadRegisters(CTRL_REG2, &Temp_CTRLREG2, 1, slave_adr, TempDeviceStatus)) { }
    while(!drvI2CReadRegisters(CTRL_REG3, &Temp_CTRLREG3, 1, slave_adr, TempDeviceStatus)) { }
    while(!drvI2CReadRegisters(CTRL_REG4, &Temp_CTRLREG4, 1, slave_adr, TempDeviceStatus)) { }
    while(!drvI2CReadRegisters(CTRL_REG5, &Temp_CTRLREG5, 1, slave_adr, TempDeviceStatus)) { }
    
   // Temp_CTRLREG1[0] = 0;
    //Temp_CTRLREG1[1] = 0;
   // MMA8452Q_SetMode(slave_adr, STANDBY); 
    
    *TempData = 0x08;              //0x39                                         // ODR = 400Hz, 
    drvI2CWriteRegisters(CTRL_REG1, TempData, 1, slave_adr, TempDeviceStatus);
    
    *TempData = 0x40;                                                               // Trigger software reset on MMA8452
    drvI2CWriteRegisters(CTRL_REG2, TempData, 1, slave_adr, TempDeviceStatus);

    delay_ms(1);          // ~1ms delay

    *TempData = 0x01;                                                               // +/-4g range with ~0.977mg/LSB  +/-2 g:0x00
    drvI2CWriteRegisters(XYZ_DATA_CFG_REG, TempData, 1, slave_adr, TempDeviceStatus);

    *TempData = 0x00;                                                               // Disable Auto Sleep, disable reset, sample mode: normal 
    drvI2CWriteRegisters(CTRL_REG2, TempData, 1, slave_adr, TempDeviceStatus);

    *TempData = 0x00;                                                               // disable FF_MT wake up, IPOL=0(interrupt polarity=active low), enable PUSH PULL
    drvI2CWriteRegisters(CTRL_REG3, TempData, 1, slave_adr, TempDeviceStatus);

    *TempData = 0x01;                                                               // DRDY interrupt routed to INT1 - PTA5
    drvI2CWriteRegisters(CTRL_REG5, TempData, 1, slave_adr, TempDeviceStatus);
    
     *TempData = 0x01;                                                               // Enable DRDY interrupt
    drvI2CWriteRegisters(CTRL_REG4, TempData, 1, slave_adr, TempDeviceStatus);
    
    
    
    MMA8452Q_SetMode(slave_adr, ACTIVE);                                         // ODR = 1.56Hz, Active mode

    while(!drvI2CReadRegisters(CTRL_REG1, &Temp_CTRLREG1, 1, slave_adr, TempDeviceStatus)) { }
    while(!drvI2CReadRegisters(CTRL_REG2, &Temp_CTRLREG2, 1, slave_adr, TempDeviceStatus)) { }
    while(!drvI2CReadRegisters(CTRL_REG3, &Temp_CTRLREG3, 1, slave_adr, TempDeviceStatus)) { }
    while(!drvI2CReadRegisters(CTRL_REG4, &Temp_CTRLREG4, 1, slave_adr, TempDeviceStatus)) { }
    while(!drvI2CReadRegisters(CTRL_REG5, &Temp_CTRLREG5, 1, slave_adr, TempDeviceStatus)) { }
     
    Temp_CTRLREG3 = 0;
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

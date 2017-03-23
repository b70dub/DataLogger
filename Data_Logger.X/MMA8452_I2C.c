/*-----------------------------------------------------------------------------
 *
 * File: MMA8452_I2C.c
 * Author: Brian Ankeny
 * PIC: 32MX440F256H @ 80MHz, 3.3v
 * MPLAB X IDE v3.35  
 * Modified From: example byTomas Vaverka, modified to run on 32MX440F256H
 *                and to support multiple MMA8452 sensors.
 *                All Rights belong to their respective owners.
 *
 *-----------------------------------------------------------------------------
*/
#include "MMA8452_Config.h"
#include "GLOBAL_VARS.h"
#include "proc/p32mx440f256h.h"

// Wrappers for MMA8452Q.

void MMA8452_Setup(UINT8 NumDevicesDetected, UINT8 NumAccels){
    UINT8 iTempCount;
    
    //Now Initialise and calibrate each MMA8452
    for(iTempCount = 1; iTempCount <= NumDevicesDetected; iTempCount++)
    {
        if((ucAddressArray[iTempCount] == 0x1C) || (ucAddressArray[iTempCount] == 0x1D)){
        ucCurrentAddress = ucAddressArray[iTempCount];
        initMMA8452Q(ucCurrentAddress);                                         //initialize
        MMA8652FC_Calibration(ucDataArray, ucCurrentAddress);                   //calibrate
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


     // Read data output registers 0x01-0x06
     while(!drvI2CReadRegisters(OUT_X_MSB_REG, ucDataArray_ref, 6, slave_adr_Copy)){}          

     // Change mode to standby before writing
     MMA8452Q_SetMode(slave_adr_Copy, STANDBY);                                 
     
     for(TempCount = 0; TempCount <= 5; TempCount++){
         ucDataArray_Temp[TempCount] = (UINT8)(*(ucDataArray_ref + TempCount));
         TempData1 = 0;
     }
     
    // Compute 12 bit output values for each axis
     Xout_12_bit = ((short) (ucDataArray_Temp[0]<<8 | ucDataArray_Temp[1])) >> 4; 
     Yout_12_bit = ((short) (ucDataArray_Temp[2]<<8 | ucDataArray_Temp[3])) >> 4; 
     Zout_12_bit = ((short) (ucDataArray_Temp[4]<<8 | ucDataArray_Temp[5])) >> 4; 

     // Compute offset correction values and write them to the device
     X_offset = Xout_12_bit / 2 * (-1);                                        
     Y_offset = Yout_12_bit / 2 * (-1);                                         
     Z_offset = (Zout_12_bit - SENSITIVITY_2G) / 2 * (-1);                      

     *TempData = X_offset;
     drvI2CWriteRegisters(OFF_X_REG, TempData, 1, slave_adr_Copy);

     *TempData = Y_offset;
     drvI2CWriteRegisters(OFF_Y_REG, TempData, 1, slave_adr_Copy);

     *TempData = Z_offset;
     drvI2CWriteRegisters(OFF_Z_REG, TempData, 1, slave_adr_Copy);


     // Change mode back to active when done
     MMA8452Q_SetMode(slave_adr_Copy, ACTIVE);                                  
}

// Sets the MMA8452Q  mode.
// 0 == STANDBY for changing registers
// 1 == ACTIVE for outputting data
void MMA8452Q_SetMode(UINT8 slave_adr, int iMode) {
    UINT8 TempData1;
    UINT8* TempData = &TempData1;
    
    if(iMode == 0)                                                              
        *TempData = 0x00;                   

    else if(iMode == 1)                                                         
        *TempData = 0x01;                                                       //  0x39: ODR = 1.56hz, 0x19 = 100hz, 0x11 = 200hz, 0x09 = 400Hz,Active mode, 0x01 = 800Hz,Active mode; 
            
    drvI2CWriteRegisters(CTRL_REG1, TempData, 1, slave_adr);
}

void initMMA8452Q(UINT8 slave_adr) {
// Initialize the MMA8452Q registers
// See the application notes for more info on setting all of these registers:

    UINT8 TempData1, Temp_CTRLREG1, Temp_CTRLREG2 = 0, Temp_CTRLREG3 = 0, Temp_CTRLREG4, Temp_CTRLREG5;
    UINT8* TempData = &TempData1;
    
    Temp_CTRLREG1 = 0;
    Temp_CTRLREG2 = 0;
    Temp_CTRLREG3 = 0;
    Temp_CTRLREG4 = 0;
    Temp_CTRLREG5 = 0;
    

    MMA8452Q_SetMode(slave_adr, STANDBY); 
    
    while(!drvI2CReadRegisters(CTRL_REG1, &Temp_CTRLREG1, 1, slave_adr)) { }
    while(!drvI2CReadRegisters(CTRL_REG2, &Temp_CTRLREG2, 1, slave_adr)) { }
    while(!drvI2CReadRegisters(CTRL_REG3, &Temp_CTRLREG3, 1, slave_adr)) { }
    while(!drvI2CReadRegisters(CTRL_REG4, &Temp_CTRLREG4, 1, slave_adr)) { }
    while(!drvI2CReadRegisters(CTRL_REG5, &Temp_CTRLREG5, 1, slave_adr)) { }
    

    // ODR = 400Hz, 
    *TempData = 0x08;              //0x39                                       
    drvI2CWriteRegisters(CTRL_REG1, TempData, 1, slave_adr);
    
    // Trigger software reset on device
    *TempData = 0x40;                                                           
    drvI2CWriteRegisters(CTRL_REG2, TempData, 1, slave_adr);

    delay_ms(1);          // ~1ms delay

    // +/-4g range with ~0.977mg/LSB  +/-2 g:0x00
    *TempData = 0x01;                                                           
    drvI2CWriteRegisters(XYZ_DATA_CFG_REG, TempData, 1, slave_adr);

    // Disable Auto Sleep, disable reset, sample mode: normal 
    *TempData = 0x00;                                                           
    drvI2CWriteRegisters(CTRL_REG2, TempData, 1, slave_adr);

    // disable FF_MT wake up, IPOL=0(interrupt polarity=active low), enable PUSH PULL
    *TempData = 0x00;                                                           
    drvI2CWriteRegisters(CTRL_REG3, TempData, 1, slave_adr);

    // DRDY interrupt routed to INT1 - PTA5
    *TempData = 0x01;                                                           
    drvI2CWriteRegisters(CTRL_REG5, TempData, 1, slave_adr);
    
    // Enable DRDY interrupt
     *TempData = 0x01;                                                          
    drvI2CWriteRegisters(CTRL_REG4, TempData, 1, slave_adr);
    
    
    // ODR = 1.56Hz, Active mode
    MMA8452Q_SetMode(slave_adr, ACTIVE);                                        

    while(!drvI2CReadRegisters(CTRL_REG1, &Temp_CTRLREG1, 1, slave_adr)) { }
    while(!drvI2CReadRegisters(CTRL_REG2, &Temp_CTRLREG2, 1, slave_adr)) { }
    while(!drvI2CReadRegisters(CTRL_REG3, &Temp_CTRLREG3, 1, slave_adr)) { }
    while(!drvI2CReadRegisters(CTRL_REG4, &Temp_CTRLREG4, 1, slave_adr)) { }
    while(!drvI2CReadRegisters(CTRL_REG5, &Temp_CTRLREG5, 1, slave_adr)) { }
     
    Temp_CTRLREG3 = 0;

    
} // initMMA8452Q

// now application specific functions

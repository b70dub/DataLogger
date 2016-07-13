#include"GLOBAL_VARS.h"
UINT8 InitComplete;
volatile UINT8 DataReady1, DataReady2, Accel1ReadStarted, Accel2ReadStarted; //Flags for accelerometer interrupts
volatile UINT8 ucDataArray[10];
volatile UINT8 Temp_INT_SOURCE_REG = 0;
volatile struct msTimer msTestCycleTimer, msLogTimer1, msLogTimer2;
volatile struct I2C_DeviceStatuses Accel1Status = {FALSE,FALSE,0,0,0,0,0,0,0,0,0,0};
volatile struct I2C_DeviceStatuses Accel2Status = {FALSE,FALSE,0,0,0,0,0,0,0,0,0,0};

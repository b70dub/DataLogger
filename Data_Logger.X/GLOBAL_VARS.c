#include"GLOBAL_VARS.h"
volatile UINT8 DataReady1, DataReady2, ReadProcessRunning; //Flags for accelerometer interrupts
volatile UINT8 ucDataArray[10];
volatile UINT8 Temp_INT_SOURCE_REG = 0;
volatile struct msTimer msTestCycleTimer, msLogTimer1, msLogTimer2;
volatile struct I2C_ReadStatuses AccelReadStatus, Accel1ReadStatus, Accel2ReadStatus;

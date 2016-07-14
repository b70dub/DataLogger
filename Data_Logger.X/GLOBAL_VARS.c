#include"GLOBAL_VARS.h"
UINT8 InitComplete;
UINT8 NumInstalledAccels; //Number of accelerometers installed
volatile UINT8 DataReady1, DataReady2, Accel1ReadStarted, Accel2ReadStarted; //Flags for accelerometer interrupts
volatile UINT8 ucDataArray[10];
volatile UINT8 Temp_INT_SOURCE_REG = 0;
volatile BOOL I2CBusCollision = FALSE;
volatile struct msTimer msTestCycleTimer, msLogTimer1, msLogTimer2;

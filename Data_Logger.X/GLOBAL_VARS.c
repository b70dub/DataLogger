#include"GLOBAL_VARS.h"
UINT InterruptCount = 0;
UINT IntStatus = 0;
UINT8 InitComplete;
UINT8 NumInstalledAccels; //Number of accelerometers installed
volatile UINT8 DataReady; //Flags for accelerometer interrupts
volatile UINT8 ucDataArray[10];
volatile UINT8 Temp_INT_SOURCE_REG = 0;
volatile BOOL I2CBusCollision = FALSE;
volatile UINT8 InterruptStatus;                                                 //0:no isr running, 
volatile struct msTimer msTestCycleTimer, msLogTimer1, msLogTimer2;

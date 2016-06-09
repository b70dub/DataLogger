/* 
 * File:   MMA8452_Config.h
 * Author: Brian
 *
 * Created on June 6, 2016, 2:24 PM
 */

#ifndef MMA8452_CONFIG_H
#define	MMA8452_CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif


///////////////////////////////////////////////
// constants for MMA8452Q i2c registers
// the slave address for this device is set in hardware. Creating a variable to save it here is helpful.
// The SparkFun breakout board defaults to 0x1D, set to 0x1C if SA0 jumper on the bottom of the board is set
#define MMA8452Q_ADDR = 0x1D // A '<< 1' is needed.  I add the '<< 1' in the helper functions.
//const MM8452Q_ADDR = 0x1C // Use this address if SA0 jumper is set.
#define STATUS           = 0x00
#define ZYXOW_BIT        = 0x7 // name_BIT == BIT position of name
#define ZYXDR_BIT        = 0x3
#define OUT_X_MSB        = 0x01
#define SYSMOD           = 0x0B
#define SYSMOD_STANDBY   = 0x00
#define SYSMOD_WAKE      = 0x01
#define SYSMOD_SLEEP     = 0x02
#define INT_SOURCE       = 0x0C
#define SRC_ASLP_BIT     = 0x7
#define SRC_FF_MT_BIT    = 0x2
#define SRC_DRDY_BIT     = 0x0
#define WHO_AM_I         = 0x0D
#define I_AM_MMA8452Q    = 0x2A // read addr WHO_AM_I, expect I_AM_MMA8452Q
#define XYZ_DATA_CFG     = 0x0E
#define FS_2G            = 0x00
#define FS_4G            = 0x01
#define FS_8G            = 0x02
#define HPF_OUT_BIT      = 0x5
#define HP_FILTER_CUTOFF = 0x0F
#define FF_MT_CFG        = 0x15
#define ELE_BIT          = 0x7
#define OAE_BIT          = 0x6
#define XYZEFE_BIT       = 0x3 // numBits == 3 (one each for XYZ)
#define XYZEFE_ALL       = 0x07 // enable all 3 bits
#define FF_MT_SRC        = 0x16
#define EA_BIT           = 0x7
#define FF_MT_THS        = 0x17
#define DBCNTM_BIT       = 0x7
#define THS_BIT          = 0x0 // numBits == 7
#define FF_MT_COUNT      = 0x18
#define ASLP_COUNT       = 0x29
#define CTRL_REG1        = 0x2A
#define ASLP_RATE_BIT    = 0x6 // numBits == 2
#define ASLP_RATE_12p5HZ = 0x1
#define ASLP_RATE_1p56HZ = 0x3
#define DR_BIT           = 0x3 // numBits == 3
#define DR_12p5HZ        = 0x5
#define DR_1p56HZ        = 0x7
#define LNOISE_BIT       = 0x2
#define F_READ_BIT       = 0x1
#define ACTIVE_BIT       = 0x0
#define CTRL_REG2        = 0x2B
#define ST_BIT           = 0x7
#define RST_BIT          = 0x6
#define SMODS_BIT        = 0x3 // numBits == 2
#define SLPE_BIT         = 0x2
#define MODS_BIT         = 0x0 // numBits == 2
#define MODS_NORMAL      = 0x00
#define MODS_LOW_POWER   = 0x03
#define CTRL_REG3        = 0x2C
#define WAKE_FF_MT_BIT   = 0x3
#define IPOL_BIT         = 0x1
#define CTRL_REG4        = 0x2D
#define INT_EN_ASLP_BIT  = 0x7
#define INT_EN_LNDPRT_BIT= 0x4
#define INT_EN_FF_MT_BIT = 0x2
#define INT_EN_DRDY_BIT  = 0x0
#define CTRL_REG5        = 0x2E


#ifdef	__cplusplus
}
#endif

#endif	/* MMA8452_CONFIG_H */


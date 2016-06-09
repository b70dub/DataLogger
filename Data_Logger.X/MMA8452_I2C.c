
#include "MMA8452_Config.h"

// helper variables for MMA8452Q. These are not const because they may have reason to change dynamically.
i2cRetryPeriod <- 1.0 // seconds to wait before retrying a failed i2c operation
maxG <- FS_4G // what scale to get G readings
i2c <- hardware.i2c89 // now can use i2c.read()
TimeNow.
///////////////////////////////////////////////
//define functions

// start with fairly generic i2c helper functions

function readBitField(val, bitPosition, numBits){ // works for 8bit registers
// bitPosition and numBits are not bounds checked
    return (val >> bitPosition) & (0x00FF >> (8 - numBits))
}

function readBit(val, bitPosition) { return readBitField(val, bitPosition, 1) }

function writeBitField(val, bitPosition, numBits, newVal) { // works for 8bit registers
// newVal is not bounds checked
    return (val & (((0x00FF >> (8 - numBits)) << bitPosition) ^ 0x00FF)) | (newVal << bitPosition)
}

function writeBit(val, bitPosition, newVal) { return writeBitField(val, bitPosition, 1, newVal) }

// Read a single byte from addressToRead and return it as a byte.  (The '[0]' causes a byte to return)
function readReg(addressToRead) {
    return readSequentialRegs(addressToRead, 1)[0]
}

// Writes a single byte (dataToWrite) into addressToWrite.  Returns error code from i2c.write
// Continue retry until success.  Caller does not need to check error code
function writeReg(addressToWrite, dataToWrite) {
    local err = null
    while (err == null) {
        err = i2c.write(MMA8452Q_ADDR << 1, format("%c%c", addressToWrite, dataToWrite))
        // server.log(format("i2c.write addr=0x%02x data=0x%02x", addressToWrite, dataToWrite))
        if (err == null) {
            server.error("i2c.write of value " + format("0x%02x", dataToWrite) + " to " + format("0x%02x", addressToWrite) + " failed.")
            imp.sleep(i2cRetryPeriod)
            server.error("retry i2c.write")
        }
    }
    return err
}

// Read numBytes sequentially, starting at addressToRead
// Continue retry until success.  Caller does not need to check error code
function readSequentialRegs(addressToRead, numBytes) {
    local data = null

    while (data == null) {
        data = i2c.read(MMA8452Q_ADDR << 1, format("%c", addressToRead), numBytes)
        if (data == null) {
            server.error("i2c.read from " + format("0x%02x", addressToRead) + " of " + numBytes + " byte" + ((numBytes > 1) ? "s" : "") + " failed.")
            imp.sleep(i2cRetryPeriod)
            server.error("retry i2c.read")
        }
    }
    return data
}

// now functions unique to MMA8452Q

function readAccelData() {
    local rawData = null // x/y/z accel register data stored here, 3 bytes
    local accelData = array(3)
    local i
    local val

    rawData = readSequentialRegs(OUT_X_MSB, 3)  // Read the three raw data registers into data array
    foreach (i, val in rawData) {
        accelData[i] = math.floor(1000.0 * ((val < 128 ? val : val - 256) / ((64 >> maxG) + 0.0)))
            // HACK: in above calc maxG just happens to be (log2(full_scale) - 1)  see: const for FS_2G, FS_4G, FS_8G
        //convert to signed integer milliGs
    }
    return accelData
}

// Reset the MMA8452Q
function MMA8452QReset() {
    local reg

    do {
        reg = readReg(WHO_AM_I)  // Read WHO_AM_I register
        if (reg == I_AM_MMA8452Q) {
            server.log("Found MMA8452Q.  Sending RST command...")
            break
        } else {
            server.error("Could not connect to MMA8452Q: WHO_AM_I reg == " + format("0x%02x", reg))
            imp.sleep(i2cRetryPeriod)
        }
    } while (true)

    // send reset command
    writeReg(CTRL_REG2, writeBit(readReg(CTRL_REG2), RST_BIT, 1))

    do {
        reg = readReg(WHO_AM_I)  // Read WHO_AM_I register
        if (reg == I_AM_MMA8452Q) {
            server.log("MMA8452Q is online!")
            break
        } else {
            server.error("Could not connect to MMA8452Q: WHO_AM_I reg == " + format("0x%02x", reg))
            imp.sleep(i2cRetryPeriod)
        }
    } while (true)
}

function MMA8452QSetActive(mode) {
    // Sets the MMA8452Q active mode.
    // 0 == STANDBY for changing registers
    // 1 == ACTIVE for outputting data
    writeReg(CTRL_REG1, writeBit(readReg(CTRL_REG1), ACTIVE_BIT, mode))
}

function initMMA8452Q() {
// Initialize the MMA8452Q registers
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
    local reg

    MMA8452QReset() // Sometimes imp card resets and MMA8452Q keeps power
    // Must be in standby to change registers
    // in STANDBY already after RESET//MMA8452QSetActive(0)

    // Set up the full scale range to 2, 4, or 8g.
    // FIXME: assumes HPF_OUT_BIT in this same register always == 0
    writeReg(XYZ_DATA_CFG, maxG)
    server.log(format("XYZ_DATA_CFG == 0x%02x", readReg(XYZ_DATA_CFG)))

    // setup CTRL_REG1
    reg = readReg(CTRL_REG1)
    reg = writeBitField(reg, ASLP_RATE_BIT, 2, ASLP_RATE_1p56HZ)
    reg = writeBitField(reg, DR_BIT, 3, DR_12p5HZ)
    // leave LNOISE_BIT as default off to save power
    // Set Fast read mode to read 8bits per xyz instead of 12bits
    reg = writeBit(reg, F_READ_BIT, 1)
    // set all CTRL_REG1 bit fields in one i2c write
    writeReg(CTRL_REG1, reg)
    server.log(format("CTRL_REG1 == 0x%02x", readReg(CTRL_REG1)))

    // setup CTRL_REG2
    reg = readReg(CTRL_REG2)
    // set Oversample mode in sleep
    reg = writeBitField(reg, SMODS_BIT, 2, MODS_LOW_POWER)
    // Enable Auto-SLEEP
    //reg = writeBit(reg, SLPE_BIT, 1)
    // Disable Auto-SLEEP
    reg = writeBit(reg, SLPE_BIT, 0)
    // set Oversample mode in wake
    reg = writeBitField(reg, MODS_BIT, 2, MODS_LOW_POWER)
    // set all CTRL_REG2 bit fields in one i2c write
    writeReg(CTRL_REG2, reg)
    server.log(format("CTRL_REG2 == 0x%02x", readReg(CTRL_REG2)))

    // setup CTRL_REG3
    reg = readReg(CTRL_REG3)
    // allow Motion to wake from SLEEP
    reg = writeBit(reg, WAKE_FF_MT_BIT, 1)
    // change Int Polarity
    reg = writeBit(reg, IPOL_BIT, 1)
    // set all CTRL_REG3 bit fields in one i2c write
    writeReg(CTRL_REG3, reg)
    server.log(format("CTRL_REG3 == 0x%02x", readReg(CTRL_REG3)))

    // setup FF_MT_CFG
    reg = readReg(FF_MT_CFG)
    // enable ELE_BIT to latch FF_MT_SRC events
    reg = writeBit(reg, ELE_BIT, 1)
    // enable Motion detection (not Free Fall detection)
    reg = writeBit(reg, OAE_BIT, 1)
    // enable on all axis x, y, and z
    reg = writeBitField(reg, XYZEFE_BIT, 3, XYZEFE_ALL)
    // set all FF_MT_CFG bit fields in one i2c write
    writeReg(FF_MT_CFG, reg)
    server.log(format("FF_MT_CFG == 0x%02x", readReg(FF_MT_CFG)))

    // setup Motion threshold to n*0.063.  (16 * 0.063 == 1G)
    writeReg(FF_MT_THS, 60) // FIXME: this is a shortcut and assumes DBCNTM_BIT is 0
    server.log(format("FF_MT_THS == 0x%02x", readReg(FF_MT_THS)))

    // setup sleep counter, the time in multiples of 320ms of no activity to enter sleep mode
    //dont' use ASLP_COUNT for now, use change in prev AccelData reading
    //writeReg(ASLP_COUNT, 10) // 10 * 320ms = 3.2 seconds

    //Enable Sleep interrupts
//    writeReg(CTRL_REG4, writeBit(readReg(CTRL_REG4), INT_EN_ASLP_BIT, 1))
    //Enable Motion interrupts
    writeReg(CTRL_REG4, writeBit(readReg(CTRL_REG4), INT_EN_FF_MT_BIT, 1))
    // Enable interrupts on every new data
    writeReg(CTRL_REG4, writeBit(readReg(CTRL_REG4), INT_EN_DRDY_BIT, 1))
    server.log(format("CTRL_REG4 == 0x%02x", readReg(CTRL_REG4)))

    MMA8452QSetActive(1)  // Set to active to start reading
} // initMMA8452Q

// now application specific functions

function pollMMA8452Q() {
    local xyz
    local reg

    while (pollMMA8452QBusy) {
        server.log("pollMMA8452QBusy collision")
        // wait herer unitl other instance of int handler is done
        // FIXME:  I never see this error, probably not neessary, just being paranoid.
    }
    pollMMA8452QBusy = true // mark as busy
    if (hardware.pin1.read() == 1) { // only react to low to high edge
//FIXME:  do we need to check status for data ready in all xyz?//log(format("STATUS == 0x%02x", readReg(STATUS)), 80)
        reg = readReg(INT_SOURCE)
        while (reg != 0x00) {
//            server.log(format("INT_SOURCE == 0x%02x", reg))
            if (readBit(reg, SRC_DRDY_BIT) == 0x1) {
                xyz = readAccelData() // this clears the SRC_DRDY_BIT
                server.log(format("%4d %4d %4d", xyz[0], xyz[1], xyz[2]))
                // do something with xyz data here
            }
            if (readBit(reg, SRC_FF_MT_BIT) == 0x1) {
                server.log("Interrupt SRC_FF_MT_BIT")
                reg = readReg(FF_MT_SRC) // this clears SRC_FF_MT_BIT
                imp.setpowersave(false) // go to low latency mode because we detected motion
            }
            if (readBit(reg, SRC_ASLP_BIT) == 0x1) {
                reg = readReg(SYSMOD) // this clears SRC_ASLP_BIT
//                server.log(format("Entering SYSMOD 0x%02x", reg))
            }
            reg = readReg(INT_SOURCE)
        } // while (reg != 0x00)
    } else {
//        server.log("INT2 LOW")
    }
    pollMMA8452QBusy = false // clear so other inst of int handler can run
} // pollMMA8452Q

////////////////////////////////////////////////////////
// first code starts here

imp.setpowersave(true) // start in low power mode.
    // Optimized for case where wakeup was caused by periodic timer, not user activity

// Register with the server
//imp.configure("MMA8452Q 1D6", [], []) // One 6-sided Die
// no in and out []s anymore, using Agent messages

// Send status to know we are alive
server.log("BOOTING  " + versionString + " " + hardware.getimpeeid() + "/" + imp.getmacaddress())
server.log("imp software version : " + imp.getsoftwareversion())

// BUGBUG: below needed until newer firmware!?  See http://forums.electricimp.com/discussion/comment/4875#Comment_2714
// imp.enableblinkup(true)

// Configure pin1 for wakeup.  Connect MMA8452Q INT2 pin to imp pin1.
hardware.pin1.configure(DIGITAL_IN_WAKEUP, pollMMA8452Q)

// set the I2C clock speed. We can do 10 kHz, 50 kHz, 100 kHz, or 400 kHz
// i2c.configure(CLOCK_SPEED_400_KHZ)
i2c.configure(CLOCK_SPEED_100_KHZ) // try to fix i2c read errors.  May need 4.7K external pull-up to go to 400_KHZ
initMMA8452Q()  // sets up code to run on interrupts from MMA8452Q

pollMMA8452Q()  // call first time to get a value on boot.

// No more code to execute so we'll sleep until an interrupt from MMA8452Q.
// Sample functions for using MMA8452Q accelerometer
// Electric Imp Device Squirrel (.nut) code
// end of code

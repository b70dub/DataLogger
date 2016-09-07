#include "legacy/spi_legacy.h"
#include "SDCard_SPI.h"
#include "SDCard_SPI.h

/**
*    Read and write a single 8-bit word to the SD/MMC card.
*     Using standard, non-buffered mode in 8 bit words.
*    **Always check SPI1RBF bit before reading the SPI2BUF register
*    **SPI1BUF is read and/or written to receive/send data
*
*    PRECONDITION: SPI bus configured, SD card selected and ready to use
*    INPUTS: byte_DataToSend = byte to transmit (or dummy byte if only a read done)
*    OUTPUTS: none
*    RETURNS:
*/

UINT8 SD_Write(UINT8 byte_DataToSend){
    
	SPI2BUF = byte_DataToSend;                    // write to buffer for TX
 
	while (!SPI2STATbits.SPIRBF);    // wait for transfer to complete
	SPI2STATbits.SPIROV = 0;        // clear any overflow.

	return SPI2BUF;                    // read the received value
}

// Not worth code defining these since they are all the same as SD_Write()
//#define SD_Read()   SD_Write( 0xFF)
//#define SD_Clock()   SD_Write( 0xFF)
//#define SD_Disable() nMEM_CS = 1; SD_Clock()
//#define SD_Enable()  nMEM_CS = 0

/**
*    Send an SPI mode command to the SD card.
*
*    PRECONDITION: SD card powered up, CRC7 table initialized.
*    INPUTS: cmd = SPI mode command to send
*            addr= 32bit address
*    OUTPUTS: none
*    RETURNS: status read back from SD card (0xFF is fault)
*    *** NOTE nMEM_CS is still low when this function exits.
*
*     expected return responses:
*   FF - timeout
*   00 - command accepted
*   01 - command received, card in idle state after RESET
*
*    R1 response codes:
*   bit 0 = Idle state
*   bit 1 = Erase Reset
*   bit 2 = Illegal command
*   bit 3 = Communication CRC error
*   bit 4 = Erase sequence error
*   bit 5 = Address error
*   bit 6 = Parameter error
*   bit 7 = Always 0
*/
UINT8 SD_SendCmd(UINT8 cmd, LBA addr){
    
	uint16_t     n;
	UINT8        res;
	UINT8     byte;
	UINT8        CRC7 = 0;

	SD_Enable();                    // enable SD card

	byte = cmd | 0x40;
	SD_Write(byte);                    // send command packet (6 bytes)
	CRC7 = CRCAdd(CRC7, byte);
	byte = addr >> 24;
	SD_Write(byte);                   // msb of the address
	CRC7 = CRCAdd(CRC7, byte);
	byte = addr >> 16;
	SD_Write(byte);
	CRC7 = CRCAdd(CRC7, byte);
	byte = addr >> 8;
	SD_Write(byte);
	CRC7 = CRCAdd(CRC7, byte);
	SD_Write(addr);                   // lsb
	CRC7 = CRCAdd(CRC7, addr);
	CRC7 = (CRC7 << 1) | 0x01;        // CRC7 always has lsb = 1

	SD_Write(CRC7);                    // Not used unless CRC mode is turned back on for SPI access.

	n = 9;                            // now wait for a response (allow for up to 8 bytes delay)
	do {
		res = SD_Read();              // check if ready   
		if (res != 0xFF)
			break;
	} while (--n > 0);

	return (res);                     // return the result that we got from the SD card.
}

void SD_InitSPI(void)
/**
*    Configure the SD card SPI bus hardware settings and software interface.
*    The SD SPI bus uses SPI1 on the PIC24FJ128GA106 chip, RP19, RP21 and RP26.
*     Using standard, non-buffered mode in 8 bit words.
*    *** Using the SD SPI mode spec settings instead of the MCHP example.
*
*    PRECONDITION: none
*    INPUTS: none - The hardware is explicitly set up here, no alternates are considered.
*    OUTPUTS: none
*    RETURNS: none.
*/
{
	nMEM_CS = 1;                    // De-select the SD card

	if (SD_CARD.ready == 1) {
		return;
	}
	// init the spi module for a slow (init) clock speed, 8 bit byte mode
	///SPI2STATbits.SPIEN = 0;               // disable SPI for configuration
    SPI2CONbits.ON = 0;                     // disable SPI for configuration  -BTA ADDED
    
	//SPI1CON1 = 0x013c;                   // Master, CKE=1; CKP=0, sample middle, prescale 1:64 (250KHz)- works
    //SPI2CON = 0x027c;                      // Master, CKE=0; CKP=1, sample end, prescale 1:64 (250KHz) - works
	//SPI2CON1 = 0x027c;                      // Master, CKE=0; CKP=1, sample end, prescale 1:64 (250KHz) - works
    //SPI2CON2 = 0x0000;                     // No buffer, no frame mode    

    SPI2CON = 0x00000A60;               // no frame mode, Continue in Idle, 32 bit mode, Master, CKE=0; CKP=1, sample end,
	//SPI2STAT = 0x8000;              // enable
    SPI2CONbits.ON = 0;             // enable  -BTA ADDED
    
	GenerateCRCTable();                // Get ready to do SD command CRC generation
}

/**
*    Discover the type and version of the installed SD card.  This routine
*    will find any SD or SDHC card and properly set it up.
*
*    PRECONDITION: none
*    INPUTS: none
*    OUTPUTS: none
*    RETURNS: 0 if successful, some other error if not.
*/
UINT8 SD_InitMedia(void){
    
	uint16_t    n;
	UINT8     res = 0;                        // If we get that far...
	uint32_t    timer;
	UINT8     cmd;
	UINT8     ReceivedData[16];                            // for when we get some data back to look at

	if (SD_CARD.ready == 1) {
		return(0);                                // done, don't do it again.
	}

	SD_Disable();                                 // 1. start with the card not selected
	for (n = 0; n<10; n++)                        // 2. send 80 clock cycles so card can init registers
		SD_Clock();
	SD_Enable();                                // 3. now select the card

	res = SD_SendCmd(RESET, 0); SD_Disable();    // 4. send a reset command and look for "IDLE"
	if (res != 1) {
		SD_Disable();
		return(LOG_FAIL);                              // card did not respond with "idle", diagnostic value
	}

	res = SD_SendCmd(SEND_IF_COND, 0x000001AA);    // 5. Check card voltage (type) for SD 1.0 or SD 2.0
	if ((res == 0xFF) || (res == 0x05)) {             // didn't respond or responded with an "illegal cmd"
		SD_CARD.version = 1;                                  // means it's an SD 1.0 or MMC card

		timer = t_1ms + 300;                               // 6. send INIT until receive a 0 or 300ms passes
		while (timer > t_1ms) {
			res = SD_SendCmd(INIT, 0);
			SD_Disable();                                     // SendSDCmd() enables SD card
			if (!res) {
				break;                                           // The card is ready
			}
		}
		if (res != 0) {
			return(LOG_FAIL);                // failed to reset.
		}
		SD_Disable();                                      // remember to disable the card
	}
	else {                                                  // need to pick up 4 bytes for v2 card voltage description
		SD_CARD.version = 2;                        // SD version 2.0 card
		for (n = 0; n<4; n++) {
			ReceivedData[n] = SD_Read();
		}                                                       // but we'll ignore it for now, we know what the card is
		SD_Disable();

		cmd = SEND_APP_OP;                        // 6. send INIT or SEND_APP_OP repeatedly until receive a 0
		timer = t_1ms + 300;                         // wait up to .3 seconds for signs of life
		res = SD_SendCmd(APP_CMD, 0); SD_Disable();    // will still be in idle mode (0x01) after this
		while (timer > t_1ms) {
			res = SD_SendCmd(cmd, 0x40000000); SD_Disable();
			if ((res & 0x0F) == 0x05) {        // ACMD41 not recognized, use CMD1
				cmd = INIT;
			}
			else {
				cmd = SEND_APP_OP;
			}
			if (!res) {
				break;
			}
		}
		if (res != 0) {
			return(LOG_FAIL);                          // failed to reset.
		}

		res = SD_SendCmd(READ_OCR, 0);            // 7. Check for capacity of the card
		if (res != 0) {
			return(LOG_FAIL);                          // error, bad thing.
		}
		for (n = 0; n<4; n++) {
			ReceivedData[n] = SD_Read();
		}
		SD_Disable();
		if (((ReceivedData[0] & 0x40) == 0x40) && (ReceivedData[0] != 0xFF)) { // check CCS bit (bit 30), PoweredUp (bit 31) set if ready.
			SD_CARD.type = 1;                            // card is high capacity, uses block addressing
		}
		else{
			SD_CARD.type = 0;                // card is low capacity, uses byte addressing
		}
	}

	SD_CARD.ready = 1;                          // successfully initialized the SD card.
	SD_CARD.db = SD_BSIZE;        // for completeness' sake

	// Get the CSD register to find the size of the card
	res = SD_SendCmd(SEND_CSD, 0);
	if (res != 0) {
		return(LOG_FAIL);
	}
	timer = t_1ms + 300;                     // wait for a response
	while (timer > t_1ms) {
		res = SD_Read();
		if (res == DATA_START) {
			break;
		}
	}
	if (res == DATA_START) {            // if it did not timeout, read a sector of data
		for (n = 0; n< 16; n++) {
			ReceivedData[n] = SD_Read();            // read the received value
		}
		// ignore CRC (for now)
		SD_Read();
		SD_Read();
		SD_Disable();
	}
	else {
		return(LOG_FAIL);
	}
	if (SD_CARD.type == 1) {            // Uses the SDHC capacity calculation
		SD_CARD.capacity = ReceivedData[9] + 1;
		SD_CARD.capacity += (uint32_t)(ReceivedData[8] << 8);
		SD_CARD.capacity += (uint32_t)(ReceivedData[7] & 0x0F) << 12;
		SD_CARD.capacity *= 524288;                // multiply by 512KB
		// (C_SIZE + 1) * 512 * 1024
		SD_CARD.numblocks = SD_CARD.capacity / SD_CARD.blksize;
	}
	else {                                // Uses the SD capacity calculation
		SD_CARD.capacity = ((uint16_t)((ReceivedData[6] & 0x03) << 10) | (uint16_t)(ReceivedData[7] << 2) | (uint16_t)((ReceivedData[8] & 0xC0) >> 6)) + 1;
		SD_CARD.capacity = SD_CARD.capacity << (((uint16_t)((ReceivedData[9] & 0x03) << 1) | (uint16_t)((ReceivedData[10] & 0x80) >> 7)) + 2);
		SD_CARD.capacity = SD_CARD.capacity << ((uint16_t)(ReceivedData[5] & 0x0F));
		// (C_SIZE +1) <<(C_SIZE_MULT + 2) <<(READ_BL_LEN), then set SET_BLOCKLEN to be 512 next.
		SD_CARD.numblocks = SD_CARD.capacity / SD_CARD.blksize;
		res = SD_SendCmd(SET_WBLEN, 0x00000200);        // Set block size to 512 bytes
		SD_Disable();
	}

	// Now kick to full speed 8MHz mode.
	SPI1STATbits.SPIEN = 0;            // disable SPI for configuration
	//    SPI1CON1 = 0x0137;              // Master, CKE=1; CKP=0, sample middle, prescale 1:4 (4MHz) all works
	//    SPI1CON1 = 0x013b;              // Master, CKE=1; CKP=0, sample middle, prescale 1:2 (8MHz) no write
	//    SPI1CON1 = 0x003b;              // Master, CKE=0; CKP=0, sample middle, prescale 1:2 (8MHz) write, no read
	//    SPI1CON1 = 0x007b;              // Master, CKE=0; CKP=1, sample middle, prescale 1:2 (8MHz) no write
	SPI1CON1 = 0x027b;              // Master, CKE=0; CKP=1, sample end, prescale 1:2 (8MHz) all works

	SPI1STATbits.SPIEN = 1;            // disable SPI for configuration    
	return(res);
}

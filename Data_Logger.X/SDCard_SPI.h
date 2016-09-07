/* 
 * File:   SDCard_SPI.h
 * Author: Brian
 *
 * Created on August 19, 2016, 2:46 PM
 */

#ifndef SDCARD_SPI_H
#define	SDCARD_SPI_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "pic32mx\include\xc.h"
   
/* DEFINITIONS */
// Not worth code defining these since they are all the same as SD_Write()
#define SD_Read()   SD_Write( 0xFF)
#define SD_Clock()   SD_Write( 0xFF)
#define SD_Disable() nMEM_CS = 1; SD_Clock()
#define SD_Enable()  nMEM_CS = 0
    
//Card commands
#define GO_IDLE_STATE       0             // CMD0 - resets card
#define SEND_OP_COND        1            // CMD1 - send operating condition
#define SEND_IF_COND        8            // CMD8 - send interface condition
#define SEND_CSD            9            // CMD9 - send card status
#define SET_BLOCKLEN        16            // CMD16 - set blocklength
#define WRITE_SINGLE_BLOCK  24            // CMD24 - write (single) block
#define APP_CMD             55            // CMD55 - next command is an application specific command, not standard
#define READ_OCR            58            // CMD58 - read the ocr register
#define CRC_ON_OFF          59            // CMD59 - turns crc on or off

#define SD_STATUS           13            // ACMD13 - send sd card status
#define SD_SEND_OP_COND     41            // ACMD41 - send operating condition

#define READ_SINGLE_BLOCK   17            // CMD17 - read single block

#define START_BLOCK         0xFE        // used to indicate the start of a data block
#define MMC_FLOATING_BUS    0xFF        // Floating bus condition(?)

    
UINT8 SD_Write(UINT8 byte_DataToSend);
UINT8 SD_SendCmd(UINT8 cmd, LBA addr);
void SD_InitSPI(void);
UINT8 SD_InitMedia(void);

typedef struct {                        // struct to describe sd card
     unsigned         ready:1;        // card is initialized or status is OK
     unsigned         writeprotect:1;        // card is write protected?
     unsigned         carddetected:1;        // card is detected?
     unsigned         type:1;        // 1 = card is high capacity(hc); 0= low capacity
     unsigned         version:1;        // card is version 1
     unsigned         no_idle:1;    // card did not GO_STATE_IDLE
     unsigned         no_opc:1;     // send_op_cond timed out
     unsigned         no_ocr:1;    // send_ocr command timed out
     unsigned         no_pup:1;    // card not powered up after init request
     unsigned         no_csd:1;     // send_CSD command timed out
     unsigned        no_w:1;        // write did not complete (timed out)
     unsigned         csd_err:1;     // card did not issue 0xFE at start of CSD data
     unsigned        w_err:1;    // write error (command not accepted)
     unsigned         r_err:1;    // read error
     unsigned        d_err:1;    // data error (data not accepted)
     unsigned        pad:2;
     unsigned int    timeout;    // timeout value when ops timeout
     unsigned long    blksize;        // card size in 512kbyte blocks (ie actual capacity is 1000 * m_size / 2 bytes)
     unsigned long    capacity;           // card size
     unsigned long    numblocks;           // card number of blocks
 } SD_CARD;





#ifdef	__cplusplus
}
#endif

#endif	/* SDCARD_SPI_H */


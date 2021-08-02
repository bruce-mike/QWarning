/*****************************************************************************
 *   fram.h:  
 *
******************************************************************************/
#ifndef __FRAM_H 
#define __FRAM_H

// FM25V01 Info
#define FRAM_MAX_ADDRESS  0x3FFF

// FM25V01 Commands
#define FRAM_CMD_WREN      0x06  // Set wrt enable latch
#define FRAM_CMD_WRDI      0x04  // Reset wrt enable latch
#define FRAM_CMD_RDSR      0x05  // Read status reg
#define FRAM_CMD_WRSR      0x01  // Write status reg
#define FRAM_CMD_READ      0x03  // Read memory data
#define FRAM_CMD_FSTRD     0x0B  // Fast read memory data
#define FRAM_CMD_WRITE     0x02  // Write memory data
#define FRAM_CMD_SLEEP     0xB9  // Enter sleep mode
#define FRAM_CMD_RDID      0x9F  // Read device ID

// FM25V01 Status register bits
#define B_FRAM_STATUS_WEL  (1 << 1)
#define B_FRAM_STATUS_BP0  (1 << 2)
#define B_FRAM_STATUS_BP1  (1 << 3)
#define B_FRAM_STATUS_WPEN (1 << 7)

// SSP1 status register bits
#define B_TFE              (1 << 0) // Tx FIFO empty
#define B_TNF              (1 << 1) // Tx FIFO not full
#define B_RNE              (1 << 2) // Rx FIFO Not empty
#define B_RFF              (1 << 3) // Rx FIFO Full
#define B_BSY              (1 << 4) // SSPx is busy Tx/Rx'ing

// Mask for SSPx Control register 0
#define M_DSS              0x0F  // Mask for # of bits to xfer
#define M_8_BIT_XFER       0x07  // xfer 8 bits
#define M_16_BIT_XFER      0x0F  // xfer 16 bits 

// Bits for SSPx interrupt Mask status & Set/Clear register
#define B_RORMIS           (1 << 0) // Rx FIFO OV
#define B_RTMIS            (1 << 1) // Rx FIFO not empty & timed out reading
#define B_RXMIS            (1 << 2) // RX FIFO at least 1/2 full
#define B_TXMIS            (1 << 3) // TX FIFO at least 1/2 empty

// Bits for clearing interrupts
#define B_RORIC            (1 << 0)
#define B_RTIC             (1 << 1)


extern void framInit( ePLATFORM_TYPE ePlatformType );
extern BOOL readFramData( uint32_t address, uint8_t *pData, uint16_t length );
extern BOOL writeFramData( uint32_t address, uint8_t *pData, uint16_t length );
extern BOOL factoryResetFramDevice( void );


#endif /* end __FRAM_H */

/*****************************************************************************
**                            End Of File
******************************************************************************/

/*****************************************************************************
 *   fram.c:  FRAM module file for FM25V01
 *   All operations are 8 bits.
 *
 *
 ******************************************************************************/
#include "LPC23xx.h"                        /* LPC23xx definitions */
#include <stdio.h>
#include <string.h>
#include "shareddefs.h"
#include "sharedInterface.h"
#include "timer.h"
#include "adc.h"
#include "irqs.h"
#include "fram.h"
#include "misc.h"

static volatile uint8_t rxFrame[8] = {0, };    // Sources up
static volatile uint8_t dummyRead;

// Forward declarations
static BOOL setStatusRegister( BOOL factory, uint8_t protectBits );
static BOOL setWriteLatch( void );
static void allowRxInterrupts( BOOL set );
static void allowTxInterrupts( BOOL set );
static void spewCmdAddress( uint8_t command, uint32_t address );
static BOOL readyForOperation( void );

/*
 * SPI interrupt.  Presently not used.  Useless.
 */
__irq void SSP1_HANDLER(void)
{
   uint8_t interruptSource;


   interruptSource = SSP1MIS;
   //printf("0x%X\r\n",interruptSource);
   do
   {
      if( ( B_RTMIS | B_RXMIS ) & interruptSource )  // Rx FIFO not empty or time out reading it
      {
         //rxFrame[rxIndex++] = SSP1DR;
         //if( sizeof( rxFrame ) / sizeof( rxFrame[0] ) <= rxIndex )
         //{
         //   rxIndex = 0;
         //}
         SSP1ICR = 0x03;//B_RTIC;             // Clear interrupt
      }
      if( B_RORMIS & interruptSource ) // Rx OV ,
      {
         (void)SSP1DR;
         SSP1ICR = B_RORIC;            // Clear interrupt
      }
      if( B_RXMIS & interruptSource )     // Should never get here
      {
         (void)SSP1DR;
      }
      if( B_TXMIS & interruptSource )     // Tx interrupt
      {
         //while( (0 < txIndex) && (SSP1SR & B_TNF) )
         //{
         //   SSP1DR = txFrame[txIndex--];     // Tx
         //}
         // Are we done?
         //if( txIndex < 0 )
         //{
         allowTxInterrupts( FALSE );
         //   setFramChipSelectHigh( );
         //}
      }

      interruptSource = SSP1MIS;
      //printf("0x%X\r\n",interruptSource);

   } while( interruptSource );

   // Clear interrupts
   SSP1ICR = 0x03;

   VICVectAddr = 0x00;
}

/*
 * Initialize SPI port using the SSP1 controller.  It has a 8 frame
 * h/w FIFO for both Tx & Rx.  Frame are typically 8 bits.
 */
void framInit( ePLATFORM_TYPE ePlatformType )
{
   uint8_t i;

   // Assumes port pins set priot to this

   enableWriteProtection( );

   // Frame = 16 bits, SPI frames, Mode 0, clk divider of 1 (1-1)
   SSP1CR0 = M_8_BIT_XFER;
   SSP1CR1 = 0;      // SSP disabled
   SSP1CPSR = 2;     // Clk divider

   // Enable SSP1
   SSP1CR1 |= ( 1 << 1 );

   for( i = 0; i < 8; i++ )
   {
      (void)SSP1DR;
   }
   // Set up interrupts
   install_irq( SSP1_INT, ( void ( * )(void) __irq )SSP1_HANDLER, HIGHEST_PRIORITY );
   // Allow Rx OV Rx not empty
   allowTxInterrupts( FALSE );
   allowRxInterrupts( FALSE );

   // Set fram status register to allow writing
   setStatusRegister( FALSE, 0 );
}

/*
 * Will read _length_ consecutive bytes.  Need pointer to place read
 * data, _*pData_ and amount of bytes _length_.
 */
BOOL readFramData( uint32_t address, uint8_t *pData, uint16_t length )
{
   // Make sure SPI port open
   if( !readyForOperation( ) )
   {
      return( FALSE );
   }

   enableWriteProtection( );

   setFramChipSelectLow( );

   spewCmdAddress( FRAM_CMD_READ, address );

   // Read data
   while( length-- )
   {
      SSP1DR = 0xFF;
      while( ( SSP1SR & B_BSY ) || !( SSP1SR & B_RNE ) );
      *pData++ = SSP1DR;
   }

   setFramChipSelectHigh( );

   return( TRUE );
}

BOOL writeFramData( uint32_t address, uint8_t *pData, uint16_t length )
{
   // Make sure SPI port open
   if( !readyForOperation( ) )
   {
      return( FALSE );
   }

   if( !setWriteLatch( ) )
   {
      return( FALSE );
   }

   disableWriteProtection( );

   setFramChipSelectLow( );

   spewCmdAddress( FRAM_CMD_WRITE, address );

   // Write data
   while( length-- )
   {
      SSP1DR = *pData++;
      while( ( SSP1SR & B_BSY ) || !( SSP1SR & B_RNE ) );
      dummyRead = SSP1DR;  // Clear Rx FIFO entry
   }

   setFramChipSelectHigh( );

   // Make sure rx FIFO empty
   while( SSP1SR & B_RNE )
   {
      rxFrame[1] = SSP1DR;
   }

   enableWriteProtection( );

   return( TRUE );
}

/*
 * Erase entire physical device, set status register back to
 * original factory default.  Takes about 35 mSecs
 */
BOOL factoryResetFramDevice( void )
{
   uint32_t address;

   // Make sure SPI port open
   if( !readyForOperation( ) )
   {
      return( FALSE );
   }

   if( !setWriteLatch( ) )
   {
      printf("Erase wre error\r\n");
      return( FALSE );
   }

   address = FRAM_MAX_ADDRESS;

   disableWriteProtection( );

   setFramChipSelectLow( );

   spewCmdAddress( FRAM_CMD_WRITE, address );

   // Fill device with...
   do
   {
      SSP1DR = 0xFF;    // Erase data
      while( ( SSP1SR & B_BSY ) || !( SSP1SR & B_RNE ) );
      dummyRead = SSP1DR; // Clear Rx FIFO entry
   } while( address-- );

   setFramChipSelectHigh( );

   // Make sure rx FIFO empty
   while( SSP1SR & B_RNE )
   {
      dummyRead = SSP1DR;
   }

   // Set status register to factory reset
   setStatusRegister( TRUE, 0 );

   enableWriteProtection( );

   return( TRUE );
}

/***** Local Functions *****/


static uint8_t readFramStatusReg( void )
{
   // Make sure SPI port open
   if( !readyForOperation( ) )
   {
      return( FALSE );
   }

   enableWriteProtection( );

   setFramChipSelectLow( );

   SSP1DR = FRAM_CMD_RDSR;
   while( ( SSP1SR & B_BSY ) || !( SSP1SR & B_RNE ) );
   dummyRead = SSP1DR;

   SSP1DR = 0xFF;
   while( ( SSP1SR & B_BSY ) || !( SSP1SR & B_RNE ) );
   dummyRead = SSP1DR;

   setFramChipSelectHigh( );

   //printf("Status Reg = 0x%X\r\n", dummyRead );
   return( dummyRead );
}

// Sets up for writes to fram, sets the WEL bit
static BOOL setWriteLatch( void )
{
   // Make sure SPI port open
   if( !readyForOperation( ) )
   {
      return( FALSE );
   }

   disableWriteProtection( );

   setFramChipSelectLow( );

   SSP1DR = FRAM_CMD_WREN;
   while( ( SSP1SR & B_BSY ) || !( SSP1SR & B_RNE ) );
   dummyRead = SSP1DR;

   // Wait till operation done
   while( SSP1SR & B_BSY );

   setFramChipSelectHigh( );

   return( TRUE );
}

// Sets up for writes to fram.  Need to sets the WPEN bit.
// If already set, (non-volatile), return.  If factory
// true then restore status register
static BOOL setStatusRegister( BOOL factory, uint8_t protectBits )
{
   uint8_t statusReg;

   statusReg = readFramStatusReg( );

   // Make sure SPI port open...again
   if( !readyForOperation( ) )
   {
      return( FALSE );
   }

   // WPEN already set & normal operation?
   if( ( statusReg & B_FRAM_STATUS_WPEN ) && ( FALSE == factory ) )
   {
      return( TRUE );
   }

   // To get here means we need to modify the status register
   if( FALSE == factory )
   {
      protectBits = ( 3 < protectBits ) ? 0 : protectBits;
      statusReg = B_FRAM_STATUS_WPEN | ( protectBits << 2 );
   }
   else
   {
      statusReg = 0;       // Factory default
   }
   // Set up for write to fram status register
   if( !setWriteLatch( ) )
   {
      return( FALSE );
   }

   disableWriteProtection( );

   setFramChipSelectLow( );

   SSP1DR = FRAM_CMD_WRSR;
   while( ( SSP1SR & B_BSY ) || !( SSP1SR & B_RNE ) );
   dummyRead = SSP1DR;

   SSP1DR = statusReg;
   while( ( SSP1SR & B_BSY ) || !( SSP1SR & B_RNE ) );
   dummyRead = SSP1DR;

   // Wait till operation done
   while( SSP1SR & B_BSY );

   setFramChipSelectHigh( );

   enableWriteProtection( );

   return( TRUE );
}

// Assumes CS is active, and !WP is set properly
// prior to this function
static void spewCmdAddress( uint8_t command, uint32_t address )
{
   SSP1DR = command;
   while( ( SSP1SR & B_BSY ) || !( SSP1SR & B_RNE ) );
   dummyRead = SSP1DR;

   SSP1DR = BYTE1( address );
   while( ( SSP1SR & B_BSY ) || !( SSP1SR & B_RNE ) );
   dummyRead = SSP1DR;

   SSP1DR = BYTE0( address );
   while( ( SSP1SR & B_BSY ) || !( SSP1SR & B_RNE ) );
   dummyRead = SSP1DR;
}

// Makes sure port is not busy, sets up for 8 bit xfer,
// presently disable all interrupts
static BOOL readyForOperation( void )
{
   uint32_t delay = 0x10000UL;

   // Make sure SPI port open
   while( SSP1SR & B_BSY )
   {
      if( 0 == delay-- )
      {
         printf("FRAM Busy Time Out\r\n");
         return( FALSE );
      }
   }

   // Tx for 8 bit frame
   SSP1CR0 = M_8_BIT_XFER;

   allowRxInterrupts( FALSE );
   allowTxInterrupts( FALSE );

   return( TRUE );
}

static void allowRxInterrupts( BOOL set )
{
   if( set )
   {
      SSP1IMSC |= ( B_RTMIS | B_RXMIS );
   }
   else
   {
      SSP1IMSC &= ~( B_RTMIS | B_RXMIS );
   }
}

static void allowTxInterrupts( BOOL set )
{
   if( set )
   {
      SSP1IMSC |= ( B_TXMIS );
   }
   else
   {
      SSP1IMSC &= ( B_TXMIS );
   }
}

/*********************************************************************************
**                            End Of File
*********************************************************************************/


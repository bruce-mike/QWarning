
#include "LPC23xx.H"                   /* LPC23xx definitions               */
#include "shareddefs.h"
#include "sharedInterface.h"
#include <stdio.h>
#include <string.h>
#include "timer.h"
#include "serial.h"
#include "irqs.h"
#include <assert.h>
#include "assetTracker.h"
#include "modem.h"
#include "radar.h"
#ifdef USING_XBEE
#include "serialWireless.h"
#endif

/*
 * There are 4 UARTS that are used on the Queue Warning project.
 * UART0 = Debug/printf 115200-8-1-N
 * UART1 = (Optional) XBee radio 9600-8-1-N
 * UART2 = Asset Tracker 115200-8-1-N
 * UART3 = Radar 1200-8-1-N
 */

/////
// rxAddToIndex is controlled by the interrupt handler
//
// rxRemoveFromIndex is controlled by the foreground process
//
//
// txRemoveFromIndex is controlled by the interrupt handler
//   with the exception of first byte added to tx queue
//   in this case, we know that the interrupt will not interfere
//
// txAddToIndex is controlled by the foreground process
/////

// UART 2 Asset Tracker/Modem
static uint16_t nTxAssetRemoveFromIndex;
static int16_t nTxAssetAddToIndex = NO_PACKET_DATA;

static uint8_t txAssetFIFO[ASSET_TX_FIFO_LENGTH];


// UART3 Radar
static uint16_t nRxRadarAddToIndex;
static uint16_t nRxRadarRemoveFromIndex;
static uint16_t nTxRadarRemoveFromIndex;
static int16_t nTxRadarAddToIndex = NO_PACKET_DATA;

static uint8_t rxRadarFIFO[RADAR_FIFO_LENGTH];
static uint8_t txRadarFIFO[RADAR_FIFO_LENGTH];
static int16_t rxRadarPacketIndex;
static uint8_t rxRadarPacket[MAX_MODEM_PACKET_LENGTH];

/*
 * Wireless serial interrupt.  This code is in this module and not in
 * the wireless directory for arbitrary reasons.
 */
__irq void UART1_HANDLER(void)
{
   #ifdef USING_XBEE
   unsigned char nData;

   /////
   // read all waiting data
   // don't do this if receive interrupts are turned off
   /////
   if( U1IER & RX_DATA_AVAILABLE_INTERRUPT_ENABLE )
   {
      while( U1LSR & RDR )        // Loop till no more Rx data
      {
         nData = U1RBR;               // Read in 1 byte
         storeByteToWirelessRxFifoIrq( nData );
         //printf("rx %X %X\r\n",U1LSR, nData );

      }
   }

   /////
   // write all available data
   /////
   if( U1IER & THRE_INTERRUPT_ENABLE )
   {
      while( U1LSR & THRE )       // Loop while Tx register is empty
      {
         if( !writeFifoByteToWirelessTxIrq( ) )
            break;
      }
   }
   #endif
   VICVectAddr = 0x00;
}


/*
 * Radar interrupt
 */
__irq void UART3_HANDLER(void)
{
   int nAddIndex;

   // Any Rx data available?
   if( U3IER & RX_DATA_AVAILABLE_INTERRUPT_ENABLE )
   {
      // While data available, read and store into FIFO
      while( U3LSR & RDR )
      {
         unsigned char nData = U3RBR;
         /////
         // don't let the add to index
         // roll past the remove from index
         // discard the data if it would
         /////
         nAddIndex = nRxRadarAddToIndex + 1;
         if( sizeof( rxRadarFIFO ) <= nAddIndex )
         {
            nAddIndex = 0;
         }
         if( nAddIndex != nRxRadarRemoveFromIndex )
         {
            rxRadarFIFO[nRxRadarAddToIndex] = nData;
            nRxRadarAddToIndex = nAddIndex;
         }
      }
   }
   #if 0 // Radar can only spew
   // Anything to Tx?
   if( U3IER & THRE_INTERRUPT_ENABLE )
   {
      // While data available to Tx xfer from FIFO to tx register.
      while( U3LSR & THRE )
      {
         if( nTxRadarRemoveFromIndex != nTxRadarAddToIndex )
         {
            unsigned char nData = txRadarFIFO[nTxRadarRemoveFromIndex++];

            U3THR = nData;

            if( sizeof( txRadarFIFO ) <= nTxRadarRemoveFromIndex )
            {
               nTxRadarRemoveFromIndex = 0;
            }
            /////
            // if this is the last character for awhile
            // so turn off thre interrupt
            // it will be re-enabled when there is something to send
            /////
            if( nTxRadarRemoveFromIndex == nTxRadarAddToIndex )
            {
               U3IER &= ( ~THRE_INTERRUPT_ENABLE );
               break;
            }
         }
         else
         {
            U3IER &= ( ~THRE_INTERRUPT_ENABLE );
            break;
         }
      }
   }
   #endif
   VICVectAddr = 0x00;
}

/*
 * Initialize indivdual ports, passed in, to the proper baud, data, etc.
 * Also set up port pins, redundent?
 */
static int serialInitPort( int nUart )
{
   int status = SUCCESS;
   /* Initialize the serial interface(s) */

   switch( nUart )
   {
      case eINTERFACE_DEBUG:
         // 115200-8-N-1
         U0LCR |= ( 1 << DLAB ) | ( EIGHTBITS << DATA_WIDTH );
         U0DLM = DLM_115200_36MHZ;
         U0DLL = DLL_115200_36MHZ;
         U0LCR &= ~( 1 << DLAB );

         // config IO pins
         PINSEL0 |= ( UART0_SEL << TXD0_SEL ) | ( UART0_SEL << RXD0_SEL );

         //U0FCR |= FIFO_ENABLE;
         break;

      case eINTERFACE_WIRELESS:
         // 9600-8-N-1
         U1LCR |= ( 1 << DLAB ) | ( EIGHTBITS << DATA_WIDTH );

         U1DLM = DLM_9600_36MHZ;
         U1DLL = DLL_9600_36MHZ;

         U1LCR &= ~( 1 << DLAB );
         // set up pins
         PINSEL0 |= ( UART1_SEL << TXD1_SEL );
         PINSEL1 |= ( UART1_SEL << RXD1_SEL );

         //U2FCR |= FIFO_ENABLE;
         break;

      case eINTERFACE_ASSET_TRACKER:
         // 115200-8-N-1
         U2LCR |= ( 1 << DLAB ) | ( EIGHTBITS << DATA_WIDTH );

         U2DLM = DLM_115200_36MHZ;
         U2DLL = DLL_115200_36MHZ;
         U2LCR &= ~( 1 << DLAB );

         // gpio.c sets up pins

         //U2FCR |= FIFO_ENABLE;
         break;

      case eINTERFACE_RADAR:
         // 1200-8-N-1
         U3LCR |= ( 1 << DLAB ) | ( EIGHTBITS << DATA_WIDTH );

         U3DLM = DLM_1200_36MHZ;
         U3DLL = DLL_1200_36MHZ;
         U3LCR &= ~( 1 << DLAB );

         // gpio.c sets up pins

         //U3FCR |= FIFO_ENABLE;
         break;
      default:
         status = ERROR;
         break;
   }
   return status;
}

/*----------------------------------------------------------------------------
*       sendchar:  Write a character to Serial Port attached to UART0
*                  called by printf through retarget.c
*---------------------------------------------------------------------------*/
int sendchar (int ch)
{
   #ifdef DEBUG_USE_INTERRUPTS
   int nRetVal = 0;

   serialDebugSendData(ch);
   return nRetVal;
   #else
   if( ch == '\n' ) {
      while( !( U0LSR & 0x20 ) );
      U0THR = '\r';
   }

   while( !( U0LSR & 0x20 ) );
   return U0THR = ch;
   #endif
}
/*----------------------------------------------------------------------------
*       RxDataReady:  if U0RBR contains valid data reurn 1, otherwise 0
*---------------------------------------------------------------------------*/
int RxDataReady( void ) {
   return ( U0LSR & 0x01 );         //Receive data ready

}

void serialDisableInterface( eINTERFACE eInterface )
{
   switch( eInterface )
   {
      case eINTERFACE_DEBUG:
         U0IER &= ~( RX_DATA_AVAILABLE_INTERRUPT_ENABLE );
         break;
      case eINTERFACE_WIRELESS:
         U1IER = 0UL;                  // Disable all interrupts

         delayMs(1);                   // Let any characters pass
         if( U1LSR ^ ( THRE | TEMT ) ) // Check for abnormal condition
         {
            (void)U1RBR;               // Clear out receive reg
         }
         U1FCR = 0x06;                 // Clear both FIFO
         break;
      case eINTERFACE_ASSET_TRACKER:
         U2IER &= ~( RX_DATA_AVAILABLE_INTERRUPT_ENABLE );
         delayMs(1);                // Let any characters pass
         if( U2LSR ^ ( THRE | TEMT ) )    // Check for abnormal condition
         {
            (void)U2RBR;                  // Clear out receive reg
         }
         U2FCR = 0x06;                    // Clear both FIFO
         U2FCR = 0xC1;                    // Enable both FIFO
         break;
      case eINTERFACE_RADAR:
         U3IER &= ~( RX_DATA_AVAILABLE_INTERRUPT_ENABLE );
         delayMs(1);                // Let any characters pass
         if( U3LSR ^ ( THRE | TEMT ) )          // Check for abnormal condition
         {
            (void)U3RBR;                  // Clear out receive reg
         }
         U3FCR = 0x06;                    // Clear both FIFO
         U3FCR = 0xC1;                    // Enable both FIFO
         break;
      default:
          printf("serialDisableInterface() - unknown port\r\n");
          break;
   }
}

void serialEnableInterface( eINTERFACE eInterface )
{
   /////
   // clear the FIFOs
   // reset the indicies
   // and enable receive interrupts
   // for the specified interface
   /////
   switch( eInterface )
   {
      case eINTERFACE_DEBUG:
         break;

      case eINTERFACE_WIRELESS:
      #ifdef USING_XBEE
         enableWirelessInterface( );
      #endif
         break;

      case eINTERFACE_ASSET_TRACKER:
         U2FCR = 0x00;                    // Disable and clear FIFO

         nTxAssetRemoveFromIndex = 0;
         nTxAssetAddToIndex = 0;
         memset( txAssetFIFO, 0, sizeof( txAssetFIFO ) );

         U2IER |= ( RX_DATA_AVAILABLE_INTERRUPT_ENABLE );
         break;

      case eINTERFACE_RADAR:
         U3FCR = 0x00;                    // Disable FIFO

         nRxRadarAddToIndex = 0;
         nRxRadarRemoveFromIndex = 0;
         nTxRadarRemoveFromIndex = 0;
         nTxRadarAddToIndex = 0;
         memset( rxRadarFIFO, 0, sizeof( rxRadarFIFO ) );
         memset( txRadarFIFO, 0, sizeof( txRadarFIFO ) );
         memset( rxRadarPacket, 0, sizeof( rxRadarPacket ) );
         rxRadarPacketIndex = NO_PACKET_DATA;

         U3IER |= ( RX_DATA_AVAILABLE_INTERRUPT_ENABLE );
         break;
      default:
         break;
   }
}


/*
 * Places 1 byte onto the Tx Asset FIFO
 */
void serialSendByteToAsset(unsigned char nData)
{
   int nAddIndex;
   BOOL bInterruptsOn = FALSE;

    
   // Turn off interrupts when messing with FIFO's
   if( U2IER & THRE_INTERRUPT_ENABLE )
   {
      U2IER &= ~THRE_INTERRUPT_ENABLE;
      bInterruptsOn = TRUE;
   }

   // Place data onto FIFO, check wrap, make sure there's room
   nAddIndex = nTxAssetAddToIndex + 1;
   if( sizeof( txAssetFIFO ) <= nAddIndex )
   {
      nAddIndex = 0;
   }
   if( nAddIndex != nTxAssetRemoveFromIndex )
   {
      txAssetFIFO[nTxAssetAddToIndex] = nData;        // Store onto fifo
      nTxAssetAddToIndex = nAddIndex;
   }

   /////
   // if fifo was empty (we know this because interrupts were off)
   // then send the first character on the fifo
   // and enable THRE interrupts
   // note: this should be the only place that
   // we enable these interrupts for this UART
   /////
   if( !bInterruptsOn )
   {
      int ch2 = txAssetFIFO[nTxAssetRemoveFromIndex++];

      // Note, we can modify the RemoveFrom index here because we know that
      // the interrupt will not interfere
      if( nTxAssetRemoveFromIndex >= sizeof( txAssetFIFO ) )
      {
         nTxAssetRemoveFromIndex = 0;
      }
      U2THR = ch2;
      U2IER |= THRE_INTERRUPT_ENABLE;
   }

   if( bInterruptsOn )
   {
      U2IER |= THRE_INTERRUPT_ENABLE;
   }
}

//===================================================
// Double buffering for Rx data
//===================================================

#define SIZE_OF_RX_BUFFER	350

typedef struct {
  uint16_t counter;
  uint8_t buffer[SIZE_OF_RX_BUFFER];
} RX_BUFFER;



//===========================================================================
// UART2 communications
// ASSET TRACKER INTERFACE
//===========================================================================

static RX_BUFFER uart2RxBuffer[2];
static uint8_t uart2DataIndex = 0;
static uint8_t uart2IsrIndex = 1;


//=========================================
void
switchUart2RxBuffers()
//=========================================
{
    U2IER &= ~(RX_DATA_AVAILABLE_INTERRUPT_ENABLE);
    
    // swap ISR and application buffers - always diffferent
    if(1 == uart2DataIndex)
    {
        uart2DataIndex = 0;
        uart2IsrIndex = 1;
    }
    else
    {
        uart2DataIndex = 1;
        uart2IsrIndex = 0;
    }
    
    U2IER |= RX_DATA_AVAILABLE_INTERRUPT_ENABLE;
}


//=============================================
void
assetSerialDoWork()
//=============================================
{
    uint16_t i;
    uint8_t byteVal;
    

    switchUart2RxBuffers();
    
 
    
    for(i = 0; i < uart2RxBuffer[uart2DataIndex].counter; i++)
    {
        byteVal = uart2RxBuffer[uart2DataIndex].buffer[i];
        
        buildRxAssetPacket( byteVal );
    }
    
  // all bytes read - set buffer input count to 0
  uart2RxBuffer[uart2DataIndex].counter = 0;

}

//===========================================
 __irq void UART2_HANDLER(void)
//===========================================
{
    uint16_t nAddIndex;

    
    // don't do this if receive interrupts are turned off
    if(U2IER & RX_DATA_AVAILABLE_INTERRUPT_ENABLE)
    {
      // Rx interrupt triggered read all waiting data
      nAddIndex = uart2RxBuffer[uart2IsrIndex].counter;
        
      while(U2LSR & RDR )
      {    
        // prevent buffser overrun - throw away extra characters
        if(nAddIndex < SIZE_OF_RX_BUFFER)
        {
          uart2RxBuffer[uart2IsrIndex].buffer[nAddIndex] = U2RBR;
          nAddIndex++;
        }            
      }
      
      uart2RxBuffer[uart2IsrIndex].counter = nAddIndex;
    }

    if(U2IER & THRE_INTERRUPT_ENABLE)
    {
      // only do this if this interrupt is enabled
      // so we don't step on foreground process        
      while(U2LSR & THRE)
      {
        if(nTxAssetRemoveFromIndex != nTxAssetAddToIndex)
        { 
          U2THR = txAssetFIFO[nTxAssetRemoveFromIndex++];
  
          if(sizeof(txAssetFIFO) <= nTxAssetRemoveFromIndex)
          {
            nTxAssetRemoveFromIndex = 0;
          }

          // if this is the last character for awhile
          // so turn off thre interrupt 
          // it will be re-enabled when there is something to send
        
          if(nTxAssetRemoveFromIndex == nTxAssetAddToIndex)
          {
            U2IER &= (~THRE_INTERRUPT_ENABLE);
            break;
          }
        }
        else
        {
          U2IER &= (~THRE_INTERRUPT_ENABLE);
          break;
        }               
      }   
            
    }

    //clear int flag??

    VICVectAddr = 0x00;
}




/*
 * Places 1 byte onto the Tx Radar FIFO
 */
void serialSendByteToRadar( unsigned char nData )
{
   int nAddIndex;
   BOOL bInterruptsOn = FALSE;

   // Turn off interrupts when messing with FIFO's
   if( U3IER & THRE_INTERRUPT_ENABLE )
   {
      U3IER &= ~THRE_INTERRUPT_ENABLE;
      bInterruptsOn = TRUE;
   }

   // Place data onto FIFO, check wrap, make sure there's room
   nAddIndex = nTxRadarAddToIndex + 1;
   if( sizeof( txRadarFIFO ) <= nAddIndex )
   {
      nAddIndex = 0;
   }
   if( nAddIndex != nTxRadarRemoveFromIndex )
   {
      txRadarFIFO[nTxRadarAddToIndex] = nData;        // Store onto fifo
      nTxRadarAddToIndex = nAddIndex;
   }

   /////
   // if fifo was empty (we know this because interrupts were off)
   // then send the first character on the fifo
   // and enable THRE interrupts
   // note: this should be the only place that
   // we enable these interrupts for this UART
   /////
   if( !bInterruptsOn )
   {
      int ch2 = txRadarFIFO[nTxRadarRemoveFromIndex++];

      // Note, we can modify the RemoveFrom index here because we know that
      // the interrupt will not interfere
      if( nTxRadarRemoveFromIndex >= sizeof( txRadarFIFO ) )
      {
         nTxRadarRemoveFromIndex = 0;
      }
      U3THR = ch2;
      U3IER |= THRE_INTERRUPT_ENABLE;
   }

   if( bInterruptsOn )
   {
      U3IER |= THRE_INTERRUPT_ENABLE;
   }
}
/*
 * RADAR ONLY
 * Will check interrupt flags to determine if any of the FIFO's, that
 * are filled/emptied within the UART interrupt, have data.  If we
 * got any rx data, buildRxRadarPacket() is called.
 */
void radarSerialDoWork( void )
{
   unsigned char nData;
   BOOL bReenableInterrupts = FALSE;

   // Save present interrupt status
   if( U3IER & RX_DATA_AVAILABLE_INTERRUPT_ENABLE )
   {
      bReenableInterrupts = TRUE;
   }

   while( 1 )
   {
      // Always disable interrupts while we are messing with the FIFO
      U3IER &= ~( RX_DATA_AVAILABLE_INTERRUPT_ENABLE );

      // Receive anything?
      if( nRxRadarAddToIndex == nRxRadarRemoveFromIndex )
      {
         break;
      }
      // Else get data from Rx fifo
      nData = rxRadarFIFO[nRxRadarRemoveFromIndex++];

      if( sizeof( rxRadarFIFO ) <= nRxRadarRemoveFromIndex )
      {
         nRxRadarRemoveFromIndex = 0;
      }
      if( bReenableInterrupts )
      {
         U3IER |= RX_DATA_AVAILABLE_INTERRUPT_ENABLE;
      }
      buildRxRadarPacket( nData, rxRadarPacket, &rxRadarPacketIndex );
      //printf("Radar:%2d 0x%2X  <%d>\r\n", nData, nData, getTimeNow( ) );
   }

   if( bReenableInterrupts )
   {
      U3IER |= RX_DATA_AVAILABLE_INTERRUPT_ENABLE;
   }
}

void serialWrite(eINTERFACE eInterface, unsigned char* pPacket, int nPacketLength)
{

   switch( eInterface )
   {
      case eINTERFACE_DEBUG:
         printf(" ");
         break;
      case eINTERFACE_WIRELESS:
      #ifdef USING_XBEE
      #ifdef PACKET_PRINT
         printf( "Outgoing serialWrite(wireless) Seq#:0x%X  Cmd:0x%X  time:%d\n",pPacket[PACKET_SEQUENCE_INDEX],
                 pPacket[PACKET_COMMAND_INDEX],getTimeNow() );
      #endif

         for( nIndex = 0; nIndex < nPacketLength; nIndex++ )
         {
            wirelessSerialSendByteToRadio(pPacket[nIndex]);
            //printf("%X ",pPacket[nIndex]);//jgr new
         }
      #endif   // USING_XBEE
         break;
      case eINTERFACE_ASSET_TRACKER:

         break;
      case eINTERFACE_MODEM:

         break;

      default:
         printf("serialWrite(Unknown eInterface) defaulting to wireless\n");
         break;
   }
}


/*
 * For trunk entering project, assume the following;
 * UART0 - Debug  115.2K-8-n-1
 * UART1 - Radio  9600-8-n-1
 * UART2 - Asset Tracker 115.2K-8-N-1
 * UART3 - Radar  1200-8-n-1
 */
void serialInit( ePLATFORM_TYPE ePlatformType )
{
   // Save platform for this file
   //eOurPlatformType = ePlatformType;

   // UART0 - Debug port
   serialInitPort( eINTERFACE_DEBUG );
   serialEnableInterface( eINTERFACE_DEBUG );

   // UART1 - Wireless radio port
   #ifdef USING_XBEE
   serialInitPort( eINTERFACE_WIRELESS );
   initSerialWireless( );


   U1IER &= ( ~THRE_INTERRUPT_ENABLE );
   install_irq( UART1_INT, ( void ( * )(void) __irq )UART1_HANDLER, HIGHEST_PRIORITY );
   serialEnableInterface( eINTERFACE_WIRELESS );
   #endif

   // UART2 - Set up asset tracker port
   serialInitPort( eINTERFACE_ASSET_TRACKER );
   U2IER &= ( ~THRE_INTERRUPT_ENABLE );
   install_irq( UART2_INT, ( void ( * )(void) __irq )UART2_HANDLER, HIGHEST_PRIORITY );
   serialEnableInterface( eINTERFACE_ASSET_TRACKER );

   // UART3 - Set up modem port
   serialInitPort( eINTERFACE_RADAR );
   U3IER &= ( ~THRE_INTERRUPT_ENABLE );
   install_irq( UART3_INT, ( void ( * )(void) __irq )UART3_HANDLER, HIGHEST_PRIORITY );
   serialEnableInterface( eINTERFACE_RADAR );

   /////
   // now, printf will work
   /////
}

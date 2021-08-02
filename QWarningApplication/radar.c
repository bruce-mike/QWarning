/*
 * Radar application.  Assumes radar delivers a 7 byte serial stream roughly
 * every ~700mS.
 *
 */
#include <stdio.h>                                          // standard I/O .h-file
#include <string.h>
#include <LPC23xx.H>                                        // LPC213x definitions
#include "shareddefs.h"
#include "sharedInterface.h"
#include "serial.h"
#include "timer.h"
#include "radar.h"
#include "misc.h"
#include "fram.h"
#include "assetTracker.h"
#include "log.h"
#include <assert.h>
#ifdef USING_XBEE
#include "serialWireless.h"
#include "cmdProcess.h"
#include "commands.h"
#endif

//#define SPEW_AVG
//#define SIMULATE_RANDOM_SPEEDS
//#define DUMP_ZERO_SPEED

// Local variables
static uint32_t packetTimeStamp;
static radarInfo_t radarInfo;


// Forward declarations
static void calcAvgSpeed( calcInfo_t *calcInfo, uint16_t backIndex );
static void loadRadarParamsFromFram( void );
static void spewAverageResults(  calcInfo_t *calcInfo, uint8_t index );
#ifdef DUMP_ZERO_SPEED
static void dumpSpeedInfo( uint8_t timeIndex );
#endif


void initRadar( void )
{
   memset( &radarInfo, 0, sizeof( radarInfo_t ) );
   loadRadarParamsFromFram( );
   
   #ifdef SIMULATE_RANDOM_SPEEDS
   printf( "WARNING SIMULATE_RANDOM_SPEEDS\r\n");
   #endif
}

/*
 * This is called from radarSerialDoWork() when the Rx FIFO is not empty.
 * It passes 1 byte at a time to this function.  This routine will
 * reassemble the raw packet.  It will check the header and trailer of
 * the packet to make sure valid, and store in radarInfo.rawSpeed[].
 *
 * nData - input character/data
 * *pPacket - pointer to rxRadarPacket
 * *pnPacketIndex - pointer to rxRadarPacketIndex
 *
 * | 0x02 | 0x84  | 0x01 | Speed | 0x01 | 0xAA | 0x03 |
 */
void buildRxRadarPacket( uint8_t nData, uint8_t *pPacket, int16_t *pnPacketIndex )
{
   static BOOL packetError = FALSE;

   // 1st byte in & no previous errors?
   if( ( NO_PACKET_DATA == *pnPacketIndex ) && !packetError )
   {
      *pnPacketIndex = 0;
      packetTimeStamp = getTimeNow( );
   }

   // To get here means normal packet data, save it
   pPacket[*pnPacketIndex] = nData;
   //printf("[%d]:0x%X  <%d>\r\n", *pnPacketIndex, nData, getTimeNow( ) );
   *pnPacketIndex += 1;
   radarInfo.readingTimedOut = 0;
   radarInfo.readingTimedOutSet = 0;

   // Check for OV
   if( RADAR_PACKET_LENGTH < *pnPacketIndex )
   {
      printf("\r\nRadar PACKET ERROR OV %d\n\r", *pnPacketIndex );
      *pnPacketIndex = NO_PACKET_DATA;
      packetError = TRUE;
      return;
   }

   // If previously, had an error, stay here till re-sync'd
   if( packetError ) 
   {
      if( ( 0x03 == pPacket[*pnPacketIndex - 1] ) &&  ( 0xAA == pPacket[*pnPacketIndex - 2] ) &&  ( 0x01 == pPacket[*pnPacketIndex - 3] ) )
      {
         *pnPacketIndex = NO_PACKET_DATA;
         packetError = FALSE;
         printf( "\r\nRadar Re-Sync  <%d>\n\r", getTimeNow( ) );
      }
      return;
   }
   
   // Check if all bytes recieved
   if( *pnPacketIndex < RADAR_PACKET_LENGTH )
   {
      if( *pnPacketIndex > 3 )
      {
         // Check if in sync
         if( ( 0x02 != pPacket[0] ) || ( 0x84 != pPacket[1] ) || ( 0x01 != pPacket[2] ) )
         {
            printf("\r\nRadar SYNC ERR 0x%X 0x%X 0x%X\n\r", pPacket[0], pPacket[1], pPacket[2] );
            *pnPacketIndex = NO_PACKET_DATA;
            packetError = TRUE;
         }
      }
      return;
   }

   // To get here means got all bytes for packet, make sure in sync
   if( ( 0x01 != pPacket[4] ) || ( 0xAA != pPacket[5] ) || ( 0x03 != pPacket[6] ) )
   {
      printf("\r\nRadar TRAIL ERR [0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X]\n\r", pPacket[0], pPacket[1], pPacket[2],
             pPacket[3], pPacket[4], pPacket[5], pPacket[6] );
      *pnPacketIndex = NO_PACKET_DATA;
      packetError = TRUE;
      return;
   }

   // To get here means valid radar packet. Save
   #ifdef SIMULATE_RANDOM_SPEEDS
   // Speed = random number between 0 - 127, average = 64, change bias 310 seconds increments
   radarInfo.rawSpeed[radarInfo.rawSpeedIndex] = (uint8_t)( getTimeNow( ) & 0x0000007FUL );
   // Every other N seconds make it all 0's
   if( 0x01UL & ( getTimeNow( ) / 35000UL ) )
   {
      radarInfo.rawSpeed[radarInfo.rawSpeedIndex++] = 0;
   }
   else if( (getTimeNow( ) / 310000UL) & 0x01UL )
   {
      radarInfo.rawSpeed[radarInfo.rawSpeedIndex++] += 10;
   }
   else
   {
      if( 10 < radarInfo.rawSpeed[radarInfo.rawSpeedIndex] )
      {
         radarInfo.rawSpeed[radarInfo.rawSpeedIndex++] -= 10;
      }
      else
      {
          radarInfo.rawSpeed[radarInfo.rawSpeedIndex++] = 0;
      }
   }
   

   #else /**************** Normal operation ***************/

   radarInfo.rawSpeed[radarInfo.rawSpeedIndex++] = ( pPacket[3] <= radarInfo.maxZeroSpeed ) ? 0 :  pPacket[3];

   #endif   // SIMULATE_RANDOM_SPEEDS
   
   radarInfo.rawSpeedIndex &= MASK_RAW_RADAR_DATA;
   //printf("0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\r\n",pPacket[0],pPacket[1],pPacket[2],pPacket[3],pPacket[4],pPacket[5],pPacket[6]);

   // Reset variables for next packet
   *pnPacketIndex = NO_PACKET_DATA;
}

/*
 * Determine if any of the averaging windows have timed out.  If so
 * calculate the required info
 */
void radarDoWork( void )
{
   calcInfo_t calcInfo;
   uint8_t saveIndex;
   uint16_t numberBackwardIndexs = 0;

   // Check for time out on radar packet stream
   if( hasTimedOut( packetTimeStamp, RADAR_PACKET_TIMEOUT_MS ) )
   {
      if( 0 == radarInfo.readingTimedOutSet )
      {
         printf( "\r\nRadar tracker PACKET TIME OUT  <%d>\n\r", getTimeNow( ) );
         // These are cleared in buildRxRadarPacket( ) when new radar byte comes in
         radarInfo.readingTimedOut = 1;
         radarInfo.readingTimedOutSet = 1;
      }
   }
   /************  Every 10 seconds calculate all time slots **************/
   else if( hasTimedOut( radarInfo.timeStampWindow[INDEX_FOR_10_SECONDS], TIME_10_SECONDS ) )
   {
      // -------------  Programable time --------------------
      saveIndex = INDEX_FOR_PROGRAMMED_SECONDS;
      numberBackwardIndexs = (uint16_t)( ( radarInfo.windowAveragingTime * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
      calcAvgSpeed( &calcInfo, numberBackwardIndexs );
      // Save results
      radarInfo.lastRadarInfo[saveIndex].avgSpeed = calcInfo.avgSpeed;
      radarInfo.lastRadarInfo[saveIndex].nonZeroSpeedCount = calcInfo.nonZeroSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].totalSpeedCount = calcInfo.totalSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].occupancyPercent = calcInfo.occupancyPercent;
      // Next time
      radarInfo.timeStampWindow[saveIndex] = getTimeNow( );
      // Debug
      spewAverageResults( &calcInfo, saveIndex );

      // -------------  300 seconds --------------------
      saveIndex = INDEX_FOR_300_SECONDS;
      numberBackwardIndexs = (uint16_t)( ( TIME_300_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
      calcAvgSpeed( &calcInfo, numberBackwardIndexs );
      // Save results
      radarInfo.lastRadarInfo[saveIndex].avgSpeed = calcInfo.avgSpeed;
      radarInfo.lastRadarInfo[saveIndex].nonZeroSpeedCount = calcInfo.nonZeroSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].totalSpeedCount = calcInfo.totalSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].occupancyPercent = calcInfo.occupancyPercent;
      // Next time
      radarInfo.timeStampWindow[saveIndex] = getTimeNow( );
      // Debug
      spewAverageResults( &calcInfo, saveIndex );

      // -------------  270 seconds --------------------
      saveIndex = INDEX_FOR_270_SECONDS;
      numberBackwardIndexs = (uint16_t)( ( TIME_270_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
      calcAvgSpeed( &calcInfo, numberBackwardIndexs );
      // Save results
      radarInfo.lastRadarInfo[saveIndex].avgSpeed = calcInfo.avgSpeed;
      radarInfo.lastRadarInfo[saveIndex].nonZeroSpeedCount = calcInfo.nonZeroSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].totalSpeedCount = calcInfo.totalSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].occupancyPercent = calcInfo.occupancyPercent;
      // Next time
      radarInfo.timeStampWindow[saveIndex] = getTimeNow( );
      // Debug
      spewAverageResults( &calcInfo, saveIndex );

      // -------------  240 seconds --------------------
      saveIndex = INDEX_FOR_240_SECONDS;
      numberBackwardIndexs = (uint16_t)( ( TIME_240_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
      calcAvgSpeed( &calcInfo, numberBackwardIndexs );
      // Save results
      radarInfo.lastRadarInfo[saveIndex].avgSpeed = calcInfo.avgSpeed;
      radarInfo.lastRadarInfo[saveIndex].nonZeroSpeedCount = calcInfo.nonZeroSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].totalSpeedCount = calcInfo.totalSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].occupancyPercent = calcInfo.occupancyPercent;
      // Next time
      radarInfo.timeStampWindow[saveIndex] = getTimeNow( );
      // Debug
      spewAverageResults( &calcInfo, saveIndex );

      // -------------  210 seconds --------------------
      saveIndex = INDEX_FOR_210_SECONDS;
      numberBackwardIndexs = (uint16_t)( ( TIME_210_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
      calcAvgSpeed( &calcInfo, numberBackwardIndexs );
      // Save results
      radarInfo.lastRadarInfo[saveIndex].avgSpeed = calcInfo.avgSpeed;
      radarInfo.lastRadarInfo[saveIndex].nonZeroSpeedCount = calcInfo.nonZeroSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].totalSpeedCount = calcInfo.totalSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].occupancyPercent = calcInfo.occupancyPercent;
      // Next time
      radarInfo.timeStampWindow[saveIndex] = getTimeNow( );
      // Debug
      spewAverageResults( &calcInfo, saveIndex );

      // -------------  180 seconds --------------------
      saveIndex = INDEX_FOR_180_SECONDS;
      numberBackwardIndexs = (uint16_t)( ( TIME_180_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
      calcAvgSpeed( &calcInfo, numberBackwardIndexs );
      // Save results
      radarInfo.lastRadarInfo[saveIndex].avgSpeed = calcInfo.avgSpeed;
      radarInfo.lastRadarInfo[saveIndex].nonZeroSpeedCount = calcInfo.nonZeroSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].totalSpeedCount = calcInfo.totalSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].occupancyPercent = calcInfo.occupancyPercent;
      // Next time
      radarInfo.timeStampWindow[saveIndex] = getTimeNow( );
      // Debug
      spewAverageResults( &calcInfo, saveIndex );

      // -------------  150 seconds --------------------
      saveIndex = INDEX_FOR_150_SECONDS;
      numberBackwardIndexs = (uint16_t)( ( TIME_150_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
      calcAvgSpeed( &calcInfo, numberBackwardIndexs );
      // Save results
      radarInfo.lastRadarInfo[saveIndex].avgSpeed = calcInfo.avgSpeed;
      radarInfo.lastRadarInfo[saveIndex].nonZeroSpeedCount = calcInfo.nonZeroSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].totalSpeedCount = calcInfo.totalSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].occupancyPercent = calcInfo.occupancyPercent;
      // Next time
      radarInfo.timeStampWindow[saveIndex] = getTimeNow( );
      // Debug
      spewAverageResults( &calcInfo, saveIndex );

      // -------------  120 seconds --------------------          
      saveIndex = INDEX_FOR_120_SECONDS;
      numberBackwardIndexs = (uint16_t)( ( TIME_120_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
      calcAvgSpeed( &calcInfo, numberBackwardIndexs );
      // Save results
      radarInfo.lastRadarInfo[saveIndex].avgSpeed = calcInfo.avgSpeed;
      radarInfo.lastRadarInfo[saveIndex].nonZeroSpeedCount = calcInfo.nonZeroSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].totalSpeedCount = calcInfo.totalSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].occupancyPercent = calcInfo.occupancyPercent;
      // Next time
      radarInfo.timeStampWindow[saveIndex] = getTimeNow( );
      // Debug
      spewAverageResults( &calcInfo, saveIndex );

      // -------------  90 seconds --------------------
      saveIndex = INDEX_FOR_90_SECONDS;
      numberBackwardIndexs = (uint16_t)( ( TIME_90_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
      calcAvgSpeed( &calcInfo, numberBackwardIndexs );
      // Save results
      radarInfo.lastRadarInfo[saveIndex].avgSpeed = calcInfo.avgSpeed;
      radarInfo.lastRadarInfo[saveIndex].nonZeroSpeedCount = calcInfo.nonZeroSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].totalSpeedCount = calcInfo.totalSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].occupancyPercent = calcInfo.occupancyPercent;
      // Next time
      radarInfo.timeStampWindow[saveIndex] = getTimeNow( );
      // Debug
      spewAverageResults( &calcInfo, saveIndex );

      // -------------  60 seconds --------------------
      saveIndex = INDEX_FOR_60_SECONDS;
      numberBackwardIndexs = (uint16_t)( ( TIME_60_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
      calcAvgSpeed( &calcInfo, numberBackwardIndexs );
      // Save results
      radarInfo.lastRadarInfo[saveIndex].avgSpeed = calcInfo.avgSpeed;
      radarInfo.lastRadarInfo[saveIndex].nonZeroSpeedCount = calcInfo.nonZeroSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].totalSpeedCount = calcInfo.totalSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].occupancyPercent = calcInfo.occupancyPercent;
      // Next time
      radarInfo.timeStampWindow[saveIndex] = getTimeNow( );
      // Debug
      spewAverageResults( &calcInfo, saveIndex );

      // -------------  30 seconds --------------------
      saveIndex = INDEX_FOR_30_SECONDS;
      numberBackwardIndexs = (uint16_t)( ( TIME_30_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
      calcAvgSpeed( &calcInfo, numberBackwardIndexs );
      // Save results
      radarInfo.lastRadarInfo[saveIndex].avgSpeed = calcInfo.avgSpeed;
      radarInfo.lastRadarInfo[saveIndex].nonZeroSpeedCount = calcInfo.nonZeroSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].totalSpeedCount = calcInfo.totalSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].occupancyPercent = calcInfo.occupancyPercent;
      // Next time
      radarInfo.timeStampWindow[saveIndex] = getTimeNow( );
      // Debug
      spewAverageResults( &calcInfo, saveIndex );

      // -------------  10 seconds --------------------
      saveIndex = INDEX_FOR_10_SECONDS;
      numberBackwardIndexs = (uint16_t)( ( TIME_10_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
      calcAvgSpeed( &calcInfo, numberBackwardIndexs );
      // Save results
      radarInfo.lastRadarInfo[saveIndex].avgSpeed = calcInfo.avgSpeed;
      radarInfo.lastRadarInfo[saveIndex].nonZeroSpeedCount = calcInfo.nonZeroSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].totalSpeedCount = calcInfo.totalSpeedCount;
      radarInfo.lastRadarInfo[saveIndex].occupancyPercent = calcInfo.occupancyPercent;
      // Next time
      radarInfo.timeStampWindow[saveIndex] = getTimeNow( );
      // Debug
      spewAverageResults( &calcInfo, saveIndex );   
   }

}

/*
 * Go backwards from present time, relative to rawSpeedIndex index's, and fill in
 * calcInfo_t; averge speed, percent occupancy, total speed count, non-zero speed count
 */
static void calcAvgSpeed( calcInfo_t *calcInfo, uint16_t backIndex )
{
   uint16_t localRawIndex;
   uint32_t sum = 0UL;
   uint32_t nonZeroCount = 0;

   // Go back in time
   localRawIndex = radarInfo.rawSpeedIndex - backIndex;
   localRawIndex &= MASK_RAW_RADAR_DATA;

   do
   {
      if( 0 != radarInfo.rawSpeed[localRawIndex] )
      {
         sum += (uint32_t)radarInfo.rawSpeed[localRawIndex];
         nonZeroCount++;
      }
      localRawIndex++;
      localRawIndex &= MASK_RAW_RADAR_DATA;
   } while( localRawIndex != radarInfo.rawSpeedIndex );

   calcInfo->nonZeroSpeedCount = nonZeroCount;

   if( 0 != nonZeroCount )
   {
      calcInfo->avgSpeed = (uint8_t)( sum / nonZeroCount );
      calcInfo->occupancyPercent = (uint8_t)(( 100UL * nonZeroCount ) / (uint32_t)backIndex );
   }
   else
   {
      calcInfo->avgSpeed = 0;
      calcInfo->occupancyPercent = 0;
   }
   calcInfo->totalSpeedCount = backIndex;
   //assert( calcInfo->totalSpeedCount >= calcInfo->nonZeroSpeedCount );
}

/*
 * When this is called it will generate the calc
 */
uint8_t getAvgSpeed( uint8_t timeIndex )
{
   return( radarInfo.lastRadarInfo[timeIndex].avgSpeed );
}

uint8_t getTotalRadarReadings( uint8_t timeIndex )
{
   return( radarInfo.lastRadarInfo[timeIndex].totalSpeedCount );
}

uint8_t getNonZeroRadarReadings( uint8_t timeIndex )
{
   return( radarInfo.lastRadarInfo[timeIndex].nonZeroSpeedCount );
}

uint8_t getOccupancyPercent( uint8_t timeIndex )
{
   return( radarInfo.lastRadarInfo[timeIndex].occupancyPercent );
}

uint8_t getMinSpeedThreshold( void )
{
   return( radarInfo.maxZeroSpeed );
}

uint16_t getAverageWindowTimeSecs( void )
{
   return( (uint16_t)( radarInfo.windowAveragingTime / 1000UL ) );
}

uint16_t getRadarOperatingStatus( void )
{
   uint16_t status = 0;
   
   status = ( 1 == radarInfo.readingTimedOut ) ? B_RADAR_TIMED_OUT : 0;
   return( status );
}

// Returns last non-zero reading fom present, within window, searching back
uint8_t getLastRadarReading( void )
{
   uint16_t index = MASK_RAW_RADAR_DATA & ( radarInfo.rawSpeedIndex - 1 );
   uint16_t j = (uint16_t)( ( TIME_120_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
   
   do
   {
      if( radarInfo.rawSpeed[index] )
      {
         break;
      }
      index = MASK_RAW_RADAR_DATA & ( index - 1 );
   } while( j-- );
   
   return( radarInfo.rawSpeed[index] );
}

/*** FRAM Functions ***/
static void loadRadarParamsFromFram( void )
{
   uint8_t temp8[4];

   // Get low speed threshold
   readFramData( MAX_ZERO_SPEED_ADDR, temp8, sizeof( radarInfo.maxZeroSpeed ) );

   radarInfo.maxZeroSpeed = ( MAX_ZERO_SPEED < temp8[0] ) ? DEFAULT_MAX_ZERO_SPEED : temp8[0];

   // Averaging window
   readFramData( AVG_TIME_MSEC_ADDR, temp8, sizeof( radarInfo.windowAveragingTime ) );
   radarInfo.windowAveragingTime = ( (uint32_t)temp8[3] << 24 ) | ( (uint32_t)temp8[2] << 16 ) |
                                   ( (uint32_t)temp8[1] << 8 )  | (uint32_t)temp8[0];

   //printf("Reading avg window time %dmS\r\n", radarInfo.windowAveragingTime );

   if( ( MAX_WINDOW_TIME_MS < radarInfo.windowAveragingTime  ) || ( MIN_WINDOW_TIME_MS > radarInfo.windowAveragingTime  ) )
   {
      radarInfo.windowAveragingTime = DEFAULT_WINDOW_TIME_MS;
   }
}

void saveRadarZeroSpeed( uint8_t speed )
{
   if( ( MAX_ZERO_SPEED < speed  ) ||  ( MIN_ZERO_SPEED > speed  ) )
   {
      speed = DEFAULT_MAX_ZERO_SPEED;
   }
   radarInfo.maxZeroSpeed = speed;
   writeFramData( MAX_ZERO_SPEED_ADDR, &speed, sizeof( speed ) );
   //printf("Saving zero velocity %dmph\r\n", speed );
}

void saveAverageWindowTime( uint16_t avgWindowTimeSecs )
{
   uint32_t timeMs = 1000UL * (uint32_t)avgWindowTimeSecs;

   if( ( MAX_WINDOW_TIME_MS < timeMs  ) || ( MIN_WINDOW_TIME_MS > timeMs  ) )
   {
      timeMs = DEFAULT_WINDOW_TIME_MS;
   }
   radarInfo.windowAveragingTime = timeMs;

   writeFramData( AVG_TIME_MSEC_ADDR, (uint8_t *)&radarInfo.windowAveragingTime, sizeof( radarInfo.windowAveragingTime ) );
   //printf("Saving averaging window %dmS\r\n", radarInfo.windowAveragingTime );
}

#ifdef SPEW_AVG
static void spewAverageResults( calcInfo_t *calcInfo, uint8_t index )
{
   uint32_t temp = ( ( (uint32_t)calcInfo->totalSpeedCount ) << Q_RADAR_SAMPLE_RATE ) / RADAR_SAMPLE_RATE;
   uint8_t i;
   
   temp += ( temp % 10 ) ? 1 : 0;

   printf( "\r\nAVG%d:[%d] %dmph(0x%X) %d/%d (0x%x/0x%X)  <%d>\r\n", temp, index, calcInfo->avgSpeed, calcInfo->avgSpeed,
           calcInfo->nonZeroSpeedCount, calcInfo->totalSpeedCount,
           calcInfo->nonZeroSpeedCount, calcInfo->totalSpeedCount,
           getTimeNow( ) );
   for( i = 0; i < RADAR_SAVED_DATA_LEN; i++ )
   {
      printf( "%d(0x%X) ", radarInfo.lastRadarInfo[i].avgSpeed, radarInfo.lastRadarInfo[i].avgSpeed);
   }
   printf("\r\n");
   for( i = 0; i < RADAR_SAVED_DATA_LEN; i++ )
   {
      printf( "<%d> ", radarInfo.timeStampWindow[i] );
   }
   printf("\r\n");
}
#else
static void spewAverageResults( calcInfo_t *calcInfo, uint8_t index )
{
}
#endif   // SPEW_AVG

#ifdef DUMP_ZERO_SPEED
 static void dumpSpeedInfo( uint8_t timeIndex )
 {
    uint16_t numberBackwardIndexs;
    uint16_t localRawIndex;
    uint16_t count = 0;
    char localBuffer[80];
    
    if( INDEX_FOR_10_SECONDS == timeIndex ) { return; }
    
    sprintf( localBuffer, "\r\n<%d>\r\n", getTimeNow( ) );
    printf( "%s", localBuffer );
    printfLog( localBuffer );

    switch( timeIndex )
    {
      case INDEX_FOR_PROGRAMMED_SECONDS:
         sprintf( localBuffer, "[Program] " );
         numberBackwardIndexs = (uint16_t)( ( radarInfo.windowAveragingTime * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
         break;
      case INDEX_FOR_300_SECONDS:
         sprintf( localBuffer, "[300] ");
         numberBackwardIndexs = (uint16_t)( ( TIME_300_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
         break;
      case INDEX_FOR_270_SECONDS:
         sprintf( localBuffer, "[270] ");
         numberBackwardIndexs = (uint16_t)( ( TIME_270_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
         break;
      case INDEX_FOR_240_SECONDS:
         sprintf( localBuffer, "[240] ");
         numberBackwardIndexs = (uint16_t)( ( TIME_240_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
         break;
      case INDEX_FOR_210_SECONDS:
         sprintf( localBuffer, "[210] ");
         numberBackwardIndexs = (uint16_t)( ( TIME_210_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
         break;
      case INDEX_FOR_180_SECONDS:
         sprintf( localBuffer, "[180] ");
         numberBackwardIndexs = (uint16_t)( ( TIME_180_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
         break;
      case INDEX_FOR_150_SECONDS:
         sprintf( localBuffer, "[150] ");
         numberBackwardIndexs = (uint16_t)( ( TIME_150_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
         break;
      case INDEX_FOR_120_SECONDS:
         sprintf( localBuffer, "[120] ");
         numberBackwardIndexs = (uint16_t)( ( TIME_120_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
         break;
      case INDEX_FOR_90_SECONDS:
         sprintf( localBuffer, "[90] ");
         numberBackwardIndexs = (uint16_t)( ( TIME_90_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
         break;
      case INDEX_FOR_60_SECONDS:
         sprintf( localBuffer, "[60] ");
         numberBackwardIndexs = (uint16_t)( ( TIME_60_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
         break;
      case INDEX_FOR_30_SECONDS:
         sprintf( localBuffer, "[30] ");
         numberBackwardIndexs = (uint16_t)( ( TIME_30_SECONDS * RADAR_SAMPLE_RATE ) / ( 256UL * 1000UL ) );
         break;
      case INDEX_FOR_10_SECONDS:
         return;
      default:
         sprintf( localBuffer, "Unknown ");
         break;
   }
   printf( "%s", localBuffer );
   printfLog( localBuffer );
   
   sprintf( localBuffer,"%d mph %d/%d ", getTotalRadarReadings( timeIndex ), getNonZeroRadarReadings( timeIndex ), 
                                         getTotalRadarReadings( timeIndex ) );
   printf("%s", localBuffer );
   printfLog( localBuffer );
   
   localRawIndex = (radarInfo.rawSpeedIndex - numberBackwardIndexs - 1) & MASK_RAW_RADAR_DATA;
   sprintf( localBuffer,"Go back %d  now-> %d   back-> %d\r\n", numberBackwardIndexs, radarInfo.rawSpeedIndex, localRawIndex );
   printf("%s", localBuffer );
   printfLog( localBuffer );         
   
   do
   {
      sprintf( localBuffer,"%d ", radarInfo.rawSpeed[localRawIndex++] );
      localRawIndex &= MASK_RAW_RADAR_DATA;
      printf("%s", localBuffer );
      printfLog( localBuffer );
      count++;
      if( 0 == (count % 50) ) 
      { 
         sprintf( localBuffer,"\r\n" );
         printf("%s", localBuffer );
      }
   } while( localRawIndex != radarInfo.rawSpeedIndex );
}
#endif   // DUMP_ZERO_SPEED

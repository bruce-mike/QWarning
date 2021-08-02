/*
 *   log.c:  .
 *
 */

#include "LPC23xx.h"                        /* LPC23xx definitions */
#include <stdio.h>
#include <string.h>
#include "shareddefs.h"
#include "sharedInterface.h"
#include "timer.h"
#include "log.h"
#include "fram.h"
#include "misc.h"
#include "watchdog.h"
#include "radar.h"
 #include "sensor.h"

// Local Variables
static BOOL loggingAllowed = FALSE;
static BOOL dumpingLog = FALSE;
static uint32_t logIndex;
static uint32_t dumpIndex = LOGGING_START_ADDR;
static uint32_t dumpLength;
static char logString[80];

// Forward declaration
static void dumpTheLog( void );


void initLogging( void )
{
   loggingAllowed = FALSE;
   dumpingLog = FALSE;
   
   readFramData( LOGGING_POINTER, (uint8_t *)&logIndex, sizeof( logIndex ) );
   dumpIndex = LOGGING_START_ADDR;
   memset( logString, 0, sizeof( logString ) );
}

char *getLogCharBuffer( void )
{
   return( logString );
}

void logDoWork( void )
{ 
   if( dumpingLog )
   {
      dumpTheLog( );
   }
}


void turnOnLogging( void )
{
   //logTimeStamp = getTimeNow( );
   loggingAllowed = TRUE;
}

void turnOffLogging( void )
{
   loggingAllowed = FALSE;
}

BOOL isLoggingEnabled( void )
{
   return( loggingAllowed );
}

/*
 * Save app non-vol, erase entire fram, restore app data.
 * NOTE:Logging turned off
 */
BOOL eraseLogging( void )
{
   uint8_t distanceLow;
   uint8_t distanceHigh;
   uint8_t loggingTime;
   uint16_t sysFlags;
   uint32_t loggingPointer;

   setRedLED( 1 );
   turnOffLogging( );

   // Read and temp save app data
   readFramData( SENSOR_QUAL_DISTANCE_LOW_ADDR, &distanceLow, sizeof( distanceLow ) );
   readFramData( SENSOR_QUAL_DISTANCE_HIGH_ADDR, &distanceHigh, sizeof( distanceHigh ) );
   readFramData( SYSTEM_FLAGS_ADDR, (uint8_t *)&sysFlags, sizeof( sysFlags ) );
   readFramData( LOGGING_TIME_SECS_ADDR, &loggingTime, sizeof( loggingTime ) );

   // Reset entire device, much quicker
   factoryResetFramDevice( );

   // Restore app data
   writeFramData( SENSOR_QUAL_DISTANCE_LOW_ADDR, &distanceLow, sizeof( distanceLow ) );
   writeFramData( SENSOR_QUAL_DISTANCE_HIGH_ADDR, &distanceHigh, sizeof( distanceHigh ) );
   writeFramData( SYSTEM_FLAGS_ADDR, (uint8_t *)&sysFlags, sizeof( sysFlags ) );
   writeFramData( LOGGING_TIME_SECS_ADDR, &loggingTime, sizeof( loggingTime ) );

   // Reset logging pointer
   loggingPointer = LOGGING_START_ADDR;
   logIndex = LOGGING_START_ADDR;
   dumpIndex = LOGGING_START_ADDR;
   writeFramData( LOGGING_POINTER, (uint8_t *)&loggingPointer, sizeof( loggingPointer ) );

   initLogging( );
   setRedLED( 0 );

   return( loggingAllowed );
}

void printfLog( char *output )
{
   uint8_t length;
   
   if( loggingAllowed )
   {
      length = strlen( output );
   
      if( LOGGING_END_ADDR <= ( logIndex + length ) )
      {
         logIndex = LOGGING_START_ADDR;
      }

      watchdogFeed( );
      writeFramData( logIndex, (uint8_t *)output, length );
      logIndex += length;
   
      writeFramData( LOGGING_POINTER, (uint8_t *)&logIndex, sizeof( logIndex ) );
   }
}
void dumpLog( uint32_t dumps )
{
   dumpLength = dumps;
   dumpingLog = TRUE;
}

static void dumpTheLog( void )
{
   uint8_t buffer[32];
   
   readFramData( dumpIndex, buffer, sizeof( buffer ) );
   dumpIndex += sizeof( buffer );

   printf( "%s", buffer );
   watchdogFeed( );
      
   if( ( 0xFF == buffer[dumpIndex] ) && ( 0xFF == buffer[dumpIndex+1] ) )
   {
      dumpingLog = FALSE;
      return;
   }
   if( ( dumpIndex - LOGGING_START_ADDR ) > dumpLength )
   {
      dumpingLog = FALSE;
      return;
   }
   if( LOGGING_END_ADDR > dumpIndex )
   {
      dumpingLog = FALSE;
      return;      
   }
}

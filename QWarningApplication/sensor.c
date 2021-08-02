/*
 * Sensor application.  Assumes sensor delivers a pulse width that
 * determines the distance.  Presently has a gain of 1 uSec = 1 millimeter.
 * Edges/interrupts of pulse width measure the time duration of
 * the pulse.  The time is converted to distance, filtered and
 * then used for a small state machine.
 */
#include <stdio.h>                                          // standard I/O .h-file
#include <string.h>
#include <stdlib.h>
#include <LPC23xx.H>                                        // LPC213x definitions
#include "shareddefs.h"
#include "sharedInterface.h"
#include "gpio.h"
#include "irqs.h"
#include "serial.h"
#include "timer.h"
#include "Sensor.h"
#include "misc.h"
#include "fram.h"
#include "assetTracker.h"
#include <assert.h>
#include "radar.h"
#include "log.h"
#ifdef USING_XBEE
#include "serialWireless.h"
#include "cmdProcess.h"
#include "commands.h"
#endif

#define FREE_RUN_SENSOR

// Local variables
static volatile uint32_t sensorRisingEdgeTimeStamp;
static volatile uint32_t sensorFallingEdgeTimeStamp;
static void (*sensorState)( uint8_t distance ) = NULL;
static sensorInfo_t sensorInfo;

#ifdef USE_BIT_BANG
static uint32_t triggeredTimeStamp;

#endif

// Fwd declarations
static uint8_t calcAverageDistance( uint32_t timeMs );
static void unknownState( uint8_t distance );
static void checkTrafficStoppedState( uint8_t distance );
static void trafficStoppedState( uint8_t measuredDistance );
static void loadSensorParamsFromFram( void );

#
/*
 * Sensor edge interrupt.  This interrupt occurs on both rising and
 * falling edge(s) of the ECHO_TX_PIN.  On both edges it saves the
 * timeStamp from Timer1.  After the falling edge it sets the int
 * occurred semaphor and non-interrupt code will determine the distance.
 */
void sensorIrqHandler( void ) __irq
{
   // Check for falling edge
   if( IO0_INT_STAT_F & ( 1 << ECHO_RX_PIN ) )
   {
      sensorFallingEdgeTimeStamp = getTimer1Count( );
      sensorInfo.pulseWidthReady = 1;            // This pulse is ready to process
   }
   // Check for rising edge
   if( IO0_INT_STAT_R & ( 1 << ECHO_RX_PIN ) )
   {
      sensorRisingEdgeTimeStamp = getTimer1Count( );
      #ifndef FREE_RUN_SENSOR
      setLowUltraSonicTrigger( );
      #endif
   }
   IO0_INT_CLR = 0xFFFFFFFF;
   IO2_INT_CLR = 0xFFFFFFFF;        // Clear all gpio interrupts

   VICVectAddr = 0x00000000;        // Dummy write to signal end of interrupt
}

/*
 * Assumes sensors output is a pulse width which is the distance
 * that the sensor is measuring.  Presently using HRXL-MaxSonar
 * product MB-7363.  It puts out 1uS/1mm distance.
 *
 * This will set up the port pins, again?, to generate an interrupt
 * on both positive and negative edges.
 * P1.8 = TRIG_TX
 * P0.17 = ECHO_RX - Pulse width
 */
void sensorInit( ePLATFORM_TYPE ePlatformType )
{
   // Pins were set up in gpio.c

   // Clear info struct
   memset( &sensorInfo, 0, sizeof( sensorInfo ) );

   // New state
   sensorState = &unknownState;

   sensorInfo.ePlatform = ePlatformType;

   loadSensorParamsFromFram( );

   sensorInfo.timeStampSensorLastReading = getTimeNow( );

   // Interrupts occur on External interrupt 3
   install_irq( EINT3_INT, ( void ( * )(void) __irq )sensorIrqHandler, LOWEST_PRIORITY );

   // Clear all pending interrupts
   IO2_INT_CLR = 0xFFFFFFFF;

   // Enable interrupts
   IO0_INT_EN_R = ( 1 << ECHO_RX_PIN );
   IO0_INT_EN_F = ( 1 << ECHO_RX_PIN );
}

/*
 * Called continuously from main() loop.  Will check to see if there's a new
 * reading from ultrasonic sensor. If no timeout from sensor and no new
 * reading this returns immediately.
 * If new reading, compute distance, save in sensorData[] array and
 * compute latest moving average distance, sensorMeasuredDistance.
 * Determine if distance threshold has been tripped.
 * NOTE: Moving average is optional
 */
void sensorDoWork( void )
{
   uint32_t sensorPulseWidth;
   uint8_t sensorMeasuredDistance;  // In 1/4 feet
   uint8_t averageDistance;         // In 1/4 feet

   // Check if no sensor reading in a while, timed out, happens at por
   if( hasTimedOut( sensorInfo.timeStampSensorLastReading, NO_SENSOR_READING_MSEC ) )
   {
      if( 0 == sensorInfo.sensorTimedOutErr )
      {
         sensorInfo.sensorTimedOutErr = 1;
         startUpUltraSonicTrigger( );
         sensorInfo.timeStampSensorLastReading = getTimeNow( );
         sensorInfo.sensorIsTriggered = 1;
         #ifdef SENSOR_PRINT
         printf( "ERROR:Sensor Timed Out  <%d>\r\n", getTimeNow( ) );
         #endif
         return;
      }
   }
   // Check if still waiting for new reading, rising and falling edges
   if( !sensorInfo.pulseWidthReady && sensorInfo.sensorIsTriggered )
   {
      return;
   }
   
   // Check if we got a new ultasonic reading?
   if( sensorInfo.pulseWidthReady && sensorInfo.sensorIsTriggered )
   {
      sensorInfo.pulseWidthReady = 0;
      #ifndef FREE_RUN_SENSOR
      sensorInfo.sensorIsTriggered = 0;
      #endif
      sensorInfo.sensorTimedOutErr = 0;
      sensorPulseWidth = sensorFallingEdgeTimeStamp - sensorRisingEdgeTimeStamp;

      // Convert pulse width to 1/4 feet resolution
      sensorMeasuredDistance = (uint8_t)( ( SENSOR_COUNT_TO_QTR_FEET * sensorPulseWidth ) >> Q_SHIFT_COUNT_TO_QTR_FEET );
      
      // Determine if object present and save
      if( ( sensorInfo.sensorTriggerDistanceLow < sensorMeasuredDistance ) && 
          ( sensorInfo.sensorTriggerDistanceHigh > sensorMeasuredDistance ) )
      {
         sensorInfo.objectPresent[sensorInfo.distanceIndex] = TRUE;
      }
      else
      {
         sensorInfo.objectPresent[sensorInfo.distanceIndex] = FALSE;
      }
      // Save distance
      sensorInfo.distanceQtrFeet[sensorInfo.distanceIndex++] = sensorMeasuredDistance;
      sensorInfo.distanceIndex &= MASK_N_SENSOR_READINGS;  // Rollover

      // Check if sensor is blocked
      averageDistance = calcAverageDistance( SENSOR_BLOCKED_TIME_MSEC );
      if( averageDistance < sensorInfo.sensorTriggerDistanceLow )
      {
         if( !sensorInfo.sensorBlockedFlag )   // 1st time?
         {
            sensorInfo.sensorBlockedFlag = 1;
            sensorInfo.timeStampSensorBlocked = getTimeNow( );
         }
         else
         {
            if( hasTimedOut( sensorInfo.timeStampSensorBlocked, SENSOR_BLOCKED_TIME_MSEC ) )
            {
               sensorInfo.sensorBlockedErr = 1;
               #ifdef SENSOR_PRINT
               if( sensorInfo.sensorBlockedErr )
               {
                  printf( "ERROR:Sensor Blocked  Avg: %s <%d>\r\n", getDistanceString( averageDistance ), getTimeNow( ) );
               }
               #endif
            }
         }
      }
      else
      {
         sensorInfo.sensorBlockedFlag = 0;
         sensorInfo.sensorBlockedErr = 0;
      }

      // Time stamp this reading
      sensorInfo.timeStampSensorLastReading = getTimeNow( );

      sensorState( sensorMeasuredDistance );   // Go to present state
      //printf("Measured distance %s\r\n", getDistanceString( sensorMeasuredDistance ) );
   }
   // Is it time for new reading?
   else if( hasTimedOut( sensorInfo.timeStampSensorLastReading, SENSOR_UPDATE_TIME_MSEC ) )
   {
      setHighUltraSonicTrigger( );
      sensorInfo.sensorIsTriggered = 1;
   }
}

/*** SENSORS STATES ***/

/*
 * Stay in here till we filled in array
 */
static void unknownState( uint8_t measuredDistance )
{
   uint8_t i;

   for( i = 0; i < sizeof( sensorInfo.distanceQtrFeet ); i++ )
   {
      if( 0 == sensorInfo.distanceQtrFeet[i] )
      {
         sensorState = &checkTrafficStoppedState;
         printf( "Switch to checkTrafficStoppedState\r\n" );
         break;
      }
   }
}
/*
 * Stay in this state until we think we have traffic stopped
 */
static void checkTrafficStoppedState( uint8_t measuredDistance )
{
   uint16_t sensorOccupiedData;
   uint16_t occPercent;
   uint8_t averageSpeed;
   
   if( hasTimedOut( sensorInfo.timeStampCheckStoppedTraffic, CHECK_FOR_STOPPED_TRAFFIC_MS ) )
   {
      sensorInfo.timeStampCheckStoppedTraffic = getTimeNow( );
      
      sensorOccupiedData = getSensorOccupancy( CHECK_FOR_STOPPED_TRAFFIC_MS );
      occPercent = ( (uint16_t)BYTE1( sensorOccupiedData ) * 100 ) / (uint16_t)BYTE0( sensorOccupiedData );

      averageSpeed = getAvgSpeed( INDEX_FOR_10_SECONDS );

      // Set alarm bit??
      if( ( 75 < occPercent ) && ( RADAR_SPEED_STOPPED > averageSpeed ) )
      {
         sensorInfo.trafficStopped = 1;
         printf( "Sensor: %d / %d  %d%%  Speed %d  <%d>\r\n", BYTE1( sensorOccupiedData ), BYTE0( sensorOccupiedData ), 
                                                            occPercent,  averageSpeed, getTimeNow( ) );
         printf( "Switch to trafficStoppedState\r\n" );
         sensorState = &trafficStoppedState;
      }
   }
}

/*
 * Stay in this state until we think we have traffic moving again
 */
static void trafficStoppedState( uint8_t measuredDistance )
{
   uint16_t sensorOccupiedData;
   uint16_t occPercent;
   uint8_t averageSpeed;
   
   if( hasTimedOut( sensorInfo.timeStampCheckStoppedTraffic, CHECK_FOR_STOPPED_TRAFFIC_MS ) )
   {
      sensorInfo.timeStampCheckStoppedTraffic = getTimeNow( );
      
      sensorOccupiedData = getSensorOccupancy( CHECK_FOR_STOPPED_TRAFFIC_MS );
      occPercent = ( (uint16_t)BYTE1( sensorOccupiedData ) * 100 ) / (uint16_t)BYTE0( sensorOccupiedData );

      averageSpeed = getAvgSpeed( INDEX_FOR_10_SECONDS );

      // Set alarm bit??
      if( ( 75 > occPercent ) && ( RADAR_SPEED_MOVING < averageSpeed ) )
      {
         sensorInfo.trafficStopped = 0;
         printf( "Sensor: %d / %d  %d%%  Speed %d  <%d>\r\n", BYTE1( sensorOccupiedData ), BYTE0( sensorOccupiedData ), 
                                                            occPercent,  averageSpeed, getTimeNow( ) );
         printf( "Switch to checkTrafficStoppedState\r\n" );
         sensorState = &checkTrafficStoppedState;
      }
   }
}

/* Will load programabble parameters from non-volatile memory, then
 * do a sanity check. If fubared, use defaults
 */
static void loadSensorParamsFromFram( void )
{
   uint32_t temp32;

   // Get qualifing feet low
   readFramData( SENSOR_QUAL_DISTANCE_LOW_ADDR, (uint8_t *)&temp32, sizeof( sensorInfo.sensorTriggerDistanceLow ) );
   
   if( ( MIN_QUALIFIER_QTR_FEET_LOW > temp32 ) || ( MAX_QUALIFIER_QTR_FEET_LOW < temp32 ) )
   {
      sensorInfo.sensorTriggerDistanceLow = DEFAULT_QUALIFIER_QTR_FEET_LOW;
   }
   else
   {
      sensorInfo.sensorTriggerDistanceLow = (uint8_t)temp32;
   }
   
   // Get qualifing feet high
   readFramData( SENSOR_QUAL_DISTANCE_HIGH_ADDR, (uint8_t *)&temp32, sizeof( sensorInfo.sensorTriggerDistanceHigh ) );

   if( ( MIN_QUALIFIER_QTR_FEET_HIGH > temp32 ) || ( MAX_QUALIFIER_QTR_FEET_HIGH < temp32 ) )
   {
      sensorInfo.sensorTriggerDistanceHigh = DEFAULT_QUALIFIER_QTR_FEET_HIGH;
   }
   else
   {
      sensorInfo.sensorTriggerDistanceHigh = (uint8_t)temp32;
   }
   
   #ifdef JEFF_DEV_MODE
   printf("WARNING: In DEV MODE\r\n");
   sensorInfo.sensorTriggerDistanceLow = 1 * 4;
   sensorInfo.sensorTriggerDistanceHigh = 5 * 4;
   #endif

   #if 0//def SENSOR_PRINT
   printf( "Sensor distance Low %s\r\n", getDistanceString( sensorInfo.sensorTriggerDistanceLow ) );
   printf( "Sensor distance High %d\r\n", getDistanceString( sensorInfo.sensorTriggerDistanceHigh ) );
   #endif
}
/*
 * Helper functions
 */
void saveSensorTriggerDistanceLow( uint8_t distanceFeet )
{
   uint8_t distance = distanceFeet;

   if( ( MIN_QUALIFIER_QTR_FEET_LOW > distance ) || ( MAX_QUALIFIER_QTR_FEET_LOW < distance ) )
   {
      distance = DEFAULT_QUALIFIER_QTR_FEET_LOW;
   }
   sensorInfo.sensorTriggerDistanceLow = distance;
   writeFramData( SENSOR_QUAL_DISTANCE_LOW_ADDR, &distance, sizeof( distance ) );
   printf("Saving trigger distance low %d.%d feet\r\n", distance >> 2, ( distance & 0x3 ) * 25 );
}

void saveSensorTriggerDistanceHigh( uint8_t distanceFeet )
{
   uint8_t distance = distanceFeet;

   if( ( MIN_QUALIFIER_QTR_FEET_HIGH > distance ) || ( MAX_QUALIFIER_QTR_FEET_HIGH < distance ) )
   {
      distance = DEFAULT_QUALIFIER_QTR_FEET_HIGH;
   }
   sensorInfo.sensorTriggerDistanceHigh = distance;
   writeFramData( SENSOR_QUAL_DISTANCE_HIGH_ADDR, &distance, sizeof( distance ) );

   printf("Saving trigger distance high %d.%d feet\r\n", distance >> 2, ( distance & 0x3 ) * 25 );
}

/*
 * Returns # occupied, high byte, and total number samples, low byte, 
 * over passed in timeMs period.  Cheezy.
 * BYTE0 = Total readings within timeMs
 * BYTE1 = N Occupied within timeMs
 * % = BYTE1/BYTE0
 */
uint16_t getSensorOccupancy( uint32_t timeMs )
{
   uint16_t i;
   uint16_t indexBack;
   uint16_t summation = 0UL;

   indexBack = ( ( (timeMs << 8) / SENSOR_UPDATE_TIME_MSEC ) + ( 1UL << 7 ) ) >> 8;
   i = ( sensorInfo.distanceIndex - indexBack ) & MASK_N_SENSOR_READINGS; 
   do
   {
      summation += sensorInfo.objectPresent[i++] ? 1 : 0;
      i &= MASK_N_SENSOR_READINGS; 
      
   } while( i != sensorInfo.distanceIndex );

   //printf("Occ %dmS  %d/%d\r\n", timeMs, summation, indexBack );
   return( ( summation << 8 ) | ( indexBack & 0x00FF ) );
}

/*
 * Returns average of saved distance array over timeMs
 */
static uint8_t calcAverageDistance( uint32_t timeMs )
{
   uint16_t i;
   uint16_t indexBack;
   uint32_t summation = 0UL;

   indexBack = ( ( (timeMs << 8) / SENSOR_UPDATE_TIME_MSEC ) + ( 1UL << 7 ) ) >> 8;
   i = ( sensorInfo.distanceIndex - indexBack ) & MASK_N_SENSOR_READINGS; 
   do
   {
      summation += (uint32_t)sensorInfo.distanceQtrFeet[i++];
      i &= MASK_N_SENSOR_READINGS; 
      
   } while( i != sensorInfo.distanceIndex );
      
   return( (uint8_t)( summation / indexBack ) );
}

uint16_t getAverageDistance( void )
{
   return( calcAverageDistance( SENSOR_UPDATE_TIME_MSEC + 1UL) );
}

uint8_t getSensorTriggerDistanceLow(  void )
{
   return( sensorInfo.sensorTriggerDistanceLow );
}

uint8_t getSensorTriggerDistanceHigh( void )
{
   return( sensorInfo.sensorTriggerDistanceHigh );
}

uint8_t getLasSensortReading( void )
{
   return( sensorInfo.distanceQtrFeet[MASK_N_SENSOR_READINGS & (sensorInfo.distanceIndex-1)] );
}

uint16_t getSensorOperatingStatus( void )
{
   uint16_t status = 0;

   status |= ( sensorInfo.sensorTimedOutErr ) ? B_SENSOR_TIMED_OUT : 0;
   status |= ( sensorInfo.sensorBlockedErr ) ? B_SENSOR_BLOCKED : 0;
   status |= ( sensorInfo.trafficStopped ) ? B_SENSOR_TRAFFIC_STOPPED : 0;
   return( status );
}

#if 0
// Get median of passed in data array of size length.
// Return median value of array, hack buble sort
static uint8_t getMedian( uint8_t *dataPtr, uint16_t length )
{
   BOOL swapped;
   uint8_t i;
   
   // Sort
   do
   {
      swapped = FALSE;
      for( i = 0; i < length - 1; i++ )
      {
         if( dataPtr[i+1] < dataPtr[i] )
         {
            uint8_t temp;
            temp = dataPtr[i];
            dataPtr[i] = dataPtr[i+1];
            dataPtr[i+1] = temp;
            swapped = TRUE;
         }
      }
   } while( swapped );
   
   return( dataPtr[length/2] );
}

void testMedian( void )
{
   uint8_t data[11];
   uint8_t i;
   
   printf("TEST MEDIAN\r\n");
   for(i=0; i<11; i++) { data[i] = (rand( ) & 0x000FF); }
   printf("["); for(i=0; i<11; i++) { printf("%d ", data[i]); } printf("]\r\n");
   printf("Median %d\r\n", getMedian( data, 11 ) );
   printf("["); for(i=0; i<11; i++) { printf("%d ", data[i]); } printf("]\r\n");

}
#endif

char *getDistanceString( uint8_t distance )
{
   static char feetString[12];
   
   sprintf( feetString, "%d.%d Ft", distance >> 2, 25 * ( distance & 0x03 ) );
   return( feetString );
}


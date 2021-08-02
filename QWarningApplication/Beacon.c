/*
 * Beacon application.
 */
#include <stdio.h>                                          // standard I/O .h-file
#include <string.h>
#include <LPC23xx.H>                                        // LPC213x definitions
#include "shareddefs.h"
#include "sharedInterface.h"
#include "gpio.h"
#include "irqs.h"
#include "serial.h"
#include "timer.h"
#include "Sensor.h"
#include "misc.h"
#include "commands.h"
#include "Beacon.h"
#include "adc.h"
#include "assetTracker.h"
#include "fram.h"
#ifdef USING_XBEE
#include "serialWireless.h"
#include "cmdProcess.h"
#include "wireless.h"
#include "cmdProcess.h"

#endif


static uint32_t distanceArray[4] = {0, };
static beaconInfo_t beaconInfo;
static void (*beaconState)( void ) = NULL;
static int8_t sensorRssi = 0;
static uint16_t sensorBatteryVoltage = 0;
static uint8_t statusSensorTriggerDistanceLow;
static uint8_t statusSensorTriggerDistanceHigh;
static uint32_t statusSensorQualTime;
static uint32_t statusSensorQualPostTime;
static uint16_t sensorOperatingStatus = 0;
static eBeaconMode_t beaconMode = eSensorActivated;

// Fwd declarations
static void beaconStateNop( void );
static void loadBeaconParamsFromFram( void );
static void checkInSync( void );







// Load defaults and non-volatile
void beaconInit( ePLATFORM_TYPE ePlatformType )
{
   memset( distanceArray, 0, sizeof( distanceArray ) );

   memset( &beaconInfo, 0, sizeof( beaconInfo_t ) );

   beaconInfo.ePlatform = ePlatformType;

   if( ePLATFORM_TYPE_BEACON_ACTIVATOR == beaconInfo.ePlatform )
   {
      loadBeaconParamsFromFram( );
   }

   beaconState = &beaconStateNop;
}

// Called continously from main loop
void beaconDoWork( void )
{
   if( ePLATFORM_TYPE_TRUCK_SENSOR == beaconInfo.ePlatform )
   {
      return;
   }

   beaconState( );

   // Check if radios are paired
   if( !areDevicesCommunicating( ) && ( 0 == beaconInfo.lostRadioComms ) && haveBeenPaired( ) )
   {
      beaconInfo.lostRadioComms = 1;
      assetSendLostRadioCommunication( );
   }
   else if( ( 1 == beaconInfo.lostRadioComms ) && areDevicesCommunicating( ) ) // Paired after not paired
   {
      beaconInfo.lostRadioComms = 0;
      assetSendHaveRadioCommunication( );
   }

   // Check beacon battery
   if( LOW_BATTERY_VOLTAGE > getBeaconBatteryVoltage( ) && ( 0 == beaconInfo.lowBeaconVoltage ) )
   {
      beaconInfo.lowBeaconVoltage = 1;
      assetSendLowBeaconBattery( );
   }
   else if( ( LOW_BATTERY_VOLTAGE <= getBeaconBatteryVoltage( ) ) && ( 1 == beaconInfo.lowBeaconVoltage ) )
   {
      beaconInfo.lowBeaconVoltage = 0;
      assetSendBeaconBatteryGood( );
   }

   // Check sensor battery
   if( areDevicesCommunicating( ) )
   {
      if( LOW_BATTERY_VOLTAGE > getSensorBatteryVoltage( ) && ( 0 == beaconInfo.lowSensorVoltage ) )
      {
         beaconInfo.lowSensorVoltage = 1;
         assetSendLowSensorBattery( );
      }
      else if( ( LOW_BATTERY_VOLTAGE <= getSensorBatteryVoltage( ) ) && ( 1 == beaconInfo.lowSensorVoltage ) )
      {
         beaconInfo.lowSensorVoltage = 0;
         assetSendSensorBatteryGood( );
      }
   }

   // Check if sensor broken, timed out
   if( ( B_SENSOR_TIMED_OUT & getSensorOperatingStatus( ) ) && ( 0 == beaconInfo.badSensor ) )
   {
      beaconInfo.badSensor = 1;
      assetSendSensorBroken( B_SENSOR_TIMED_OUT );
   }
   else if( !( B_SENSOR_TIMED_OUT & getSensorOperatingStatus( ) ) && ( 1 == beaconInfo.badSensor ) )
   {
      beaconInfo.badSensor = 0;
      assetSendSensorWorking( B_SENSOR_TIMED_OUT );
   }

   // Check if sensor is blocked
   if( ( B_SENSOR_BLOCKED & getSensorOperatingStatus( ) ) && ( 0 == beaconInfo.blockedSensor ) )
   {
      beaconInfo.blockedSensor = 1;
      assetSendSensorBroken( B_SENSOR_BLOCKED );
   }
   else if( !( B_SENSOR_BLOCKED & getSensorOperatingStatus( ) ) && ( 1 == beaconInfo.blockedSensor ) )
   {
      beaconInfo.blockedSensor = 0;
      assetSendSensorWorking( B_SENSOR_BLOCKED );
   }

   // Check if lost comms
   checkInSync( );
}

static void beaconStateNop( void )
{

}

/* Stay in this state until there are no trucks and the beacon activation
 * time has expired.  Note: timeStampTruckExited is set when received a
 * truck not present command from the sensor.
 * If in this state for longer than
 */
static void beaconStateActive( void )
{
   if( beaconInfo.truckExitingActive )
   {
      if( hasTimedOut( beaconInfo.timeStampTruckExited, beaconInfo.timeBeaconActive ) )
      {
         setDryOut0( 0 );
         setAuxLED0( 0 );

         beaconInfo.truckEnteringActive = 0;
         beaconInfo.truckExitingActive = 0;
         beaconInfo.truckStuck = 0;

         // Send message up to asset tracker
         assetSendBeaconOff( );
         beaconState = &beaconStateNop;
      }
   }
   // Got here because have not yet received truck not present from sensor,
   else if( hasTimedOut( beaconInfo.timeStampTruckEntered, beaconInfo.timeTruckStuckMilliSecs ) &&
            ( 0 == beaconInfo.truckStuck ) )
   {
      beaconInfo.truckStuck = 1;

      // Send up to FM
      assetSendTruckStuckBeaconOn( );
      beaconInfo.timeStampTruckStuck = getTimerNow( );
   }
}

/*
 * Runs continuously from DoWork loop. Checks that the status of the
 * sensor and the status of the beacon/asset tracker are the same.
 */
static void checkInSync( void )
{
   uint16_t sensorStatus;
   uint16_t alarmBits;

   // Bail if no new info
   if( !( B_SENSOR_STATUS_NEW & getSensorOperatingStatus( ) ) )
   {
      return;
   }

   // Read latest sensor status and check if = to asset's last bits
   sensorStatus = getSensorOperatingStatus( );
   alarmBits = getAssetAlarmBits( );

   // Debug
   if( ( B_SENSOR_BLOCKED | B_SENSOR_TIMED_OUT | B_TRUCK_PRESENT ) & ( sensorStatus ^ alarmBits ) )
   {
      printf("OUT OF SYNC ERR: 0x%X 0x%X\r\n", sensorStatus, alarmBits );
   }

   if( B_SENSOR_BLOCKED & ( sensorStatus ^ alarmBits ) )
   {
      // To get here means out of sync
      if( B_SENSOR_BLOCKED & sensorStatus )
      {
         assetSendSensorBroken( B_SENSOR_BLOCKED );
      }
      else
      {
         assetSendSensorWorking( B_SENSOR_BLOCKED );
      }
   }
   if( B_SENSOR_TIMED_OUT & ( sensorStatus ^ alarmBits ) )
   {
      // To get here means out of sync
      if( B_SENSOR_TIMED_OUT & sensorStatus )
      {
         assetSendSensorBroken( B_SENSOR_TIMED_OUT );
      }
      else
      {
         assetSendSensorWorking( B_SENSOR_TIMED_OUT );
      }
   }
   if( B_TRUCK_PRESENT & ( sensorStatus ^ alarmBits ) )
   {
      // To get here means maybe out of sync
      if( B_TRUCK_PRESENT & sensorStatus )
      {
         gotRxTruckPresent( getSensorDistance( ) );
      }
      else
      {
         if( !beaconInfo.truckExitingActive )
         {
            gotRxTruckNotPresent( getSensorDistance( ) );
         }
      }
   }

   clearSensorOperatingStatusRxBit( );
}

/*
 * Called from commands.c, will start truck present state machine.
 * Will get new time stamp on every call and send up to asset new
 * message every time.
 */
void gotRxTruckPresent( uint32_t distance )
{
   uint32_t framTruckCounter;

   beaconInfo.timeStampTruckEntered = getTimerNow( );

   setDryOut0( 1 );
   setAuxLED0( 1 );

   beaconInfo.truckEnteringActive = 1;
   beaconInfo.truckExitingActive = 0;
   beaconInfo.truckStuck = 0;

   beaconInfo.newTruckCounter++;
   beaconState = &beaconStateActive;

   #ifdef BEACON_PRINT
   printf("BeaconRx: Truck IS Present "); printDistance( distance, TRUE );
   #endif

   // Send up to asset
   assetSendTruckPresentBeaconOn( distance );

   // Save truck counter to non-volatile
   readFramData( BEACON_ACTIVITY_COUNT_ADDR, (uint8_t *)&framTruckCounter, sizeof( framTruckCounter ) );
   if( 5 < beaconInfo.newTruckCounter - framTruckCounter )
   {
      writeFramData( BEACON_ACTIVITY_COUNT_ADDR, (uint8_t *)&beaconInfo.newTruckCounter, sizeof( beaconInfo.newTruckCounter ) );
   }
}

// reset the truck event counter, sent from the asset tracker
void resetEventCount( void )
{
   beaconInfo.newTruckCounter = 0;
   writeFramData( BEACON_ACTIVITY_COUNT_ADDR, (uint8_t *)&beaconInfo.newTruckCounter, sizeof( beaconInfo.newTruckCounter ) );
}

void setBeaconMode( eBeaconMode_t mode )
{
   if( getManualOverride( ) )
   {
      beaconMode = eManualOverRideOn;
      setDryOut0( 1 );
   }
   else
   {
      switch( mode )
      {
      case eManualOverRideOn:
         beaconMode = eManualOverRideOn;
         setDryOut0( 1 );
         break;
      case eOn:
         beaconMode = eOn;
         setDryOut0( 1 );
         break;
      case eSensorActivated:
         beaconMode = eSensorActivated;
         setDryOut0( 0 );
         break;
      case eOff:
         beaconMode = eOff;
         setDryOut0( 0 );
         break;
      default:
         printf("ERROR:Bad beacon mode\r\n");
         break;
      }
   }
   #ifdef BEACON_PRINT
   printf("New Beacon Mode ");
   if( eManualOverRideOn == beaconMode ) { printf("manualOVERRIDEon\r\n"); }
   else if( eOn == beaconMode ) { printf("ON\r\n"); }
   else if( eSensorActivated == beaconMode ) { printf("sensorACTIVATED\r\n"); }
   else if( eOff == beaconMode ) { printf("OFF\r\n"); }
   else { printf("UNKNOWN\r\n"); }
   #endif
}

/*
 * Called from commands.c, will set time stamp, timeStampTruckExited,
 * that determines when to shut down beacon.
 */
void gotRxTruckNotPresent( uint32_t distance )
{
   beaconInfo.timeStampTruckExited = getTimerNow( );
   beaconInfo.truckExitingActive = 1;
   #ifdef BEACON_PRINT
   printf("BeaconRx: Truck NOT Present "); printDistance( distance, TRUE );
   #endif
}

// Called from commands.c, saves latest sensor distance
// measurement into array, for averaging.
void saveLatestSensorDistance( uint32_t distance )
{
   static uint8_t index = 0;

   distanceArray[index++] = distance;
   if( sizeof( distanceArray ) / sizeof( distanceArray[0] ) <= index )
   {
      index = 0;
   }
}

// Returns average of latest distance measurments.
uint32_t getSensorDistance( void )
{
   uint32_t sum = 0UL;
   uint8_t i;

   for( i = 0; i < sizeof( distanceArray ) / sizeof( distanceArray[0] ); i++ )
   {
      sum += distanceArray[i];
   }
   return( sum / ( sizeof( distanceArray ) / sizeof( distanceArray[0] ) ) );
}

uint32_t getTruckEventCounter( void )
{
   return( beaconInfo.newTruckCounter );
}

/*
 * RSSI Routines for beacon radio
 */
void saveSensorRssi( int8_t rssi )
{
   sensorRssi = rssi;
}

int8_t getSensorRssi( void )
{
   return( sensorRssi );
}

int8_t getBeaconRssi( void )
{
   return( getRssi( ) );
}

/*
 * BATTERY Routines for beacon board
 * Voltages are averaged in adc routine.
 */
void saveSensorBatterVoltage( uint16_t voltage )
{
   sensorBatteryVoltage = voltage;
}

uint16_t getSensorBatteryVoltage( void )
{
   return( sensorBatteryVoltage );
}

uint16_t getBeaconBatteryVoltage( void )
{
   return( ADCGetBatteryVoltage( ) );
}

/*
 * Save programmable parameters
 */
void saveBeaconTruckStuckTime( uint32_t mSecs )
{
   if( ( MIN_TRUCK_STUCK_TIME > mSecs ) || ( MAX_TRUCK_STUCK_TIME < mSecs ) )
   {
      mSecs = DEFAULT_TRUCK_STUCK_TIME;
   }

   beaconInfo.timeTruckStuckMilliSecs = mSecs;
   writeFramData( BEACON_STUCK_TIME_ADDR, (uint8_t *)&mSecs, sizeof( mSecs ) );

   printf("Beacon:Truck stuck qualifing time %d mSecs\r\n", mSecs);
}

void saveBeaconOnTime( uint32_t mSecs )
{
   if( ( MIN_BEACON_ON_TIME > mSecs ) || ( MAX_BEACON_ON_TIME < mSecs ) )
   {
      mSecs = DEFAULT_BEACON_ON_TIME;
   }
   beaconInfo.timeBeaconActive = mSecs;
   writeFramData( BEACON_ON_TIME_ADDR, (uint8_t *)&mSecs, sizeof( mSecs ) );

   printf("Beacon:Beacon ON time %d mSecs\r\n", mSecs);
}

uint32_t getBeaconTruckStuckTime( void )
{
   return( beaconInfo.timeTruckStuckMilliSecs );
}

uint32_t getBeaconOnTime( void )
{
   return( beaconInfo.timeBeaconActive );
}

eBeaconMode_t getBeaconMode( void )
{
   return( beaconMode );
}

/* Will load programabble parameters from non-volatile memory, then
 * do a sanity check. If fubared, use defaults
 */
static void loadBeaconParamsFromFram( void )
{
   uint32_t temp32;

   readFramData( BEACON_ON_TIME_ADDR, (uint8_t *)&temp32, sizeof( temp32 ) );
   if( ( MIN_BEACON_ON_TIME > temp32 ) || ( MAX_BEACON_ON_TIME < temp32 ) )
   {
      beaconInfo.timeBeaconActive = DEFAULT_BEACON_ON_TIME;
   }
   else
   {
      beaconInfo.timeBeaconActive = temp32;
   }

   readFramData( BEACON_STUCK_TIME_ADDR, (uint8_t *)&temp32, sizeof( temp32 ) );
   if( ( MIN_TRUCK_STUCK_TIME > temp32 ) || ( MAX_TRUCK_STUCK_TIME < temp32 ) )
   {
      beaconInfo.timeTruckStuckMilliSecs = DEFAULT_TRUCK_STUCK_TIME;
   }
   else
   {
      beaconInfo.timeTruckStuckMilliSecs = temp32;
   }

   readFramData( BEACON_ACTIVITY_COUNT_ADDR, (uint8_t *)&temp32, sizeof( beaconInfo.newTruckCounter ) );
   beaconInfo.newTruckCounter = temp32;


   #ifdef JEFF_DEV_MODE
   printf("WARNING: In DEV MODE\r\n<Original parameters>\r\n");
   printf("Beacon On Time %d mSecs\r\n", beaconInfo.timeBeaconActive );
   printf("Beacon Stuck Time %d mSecs\r\n", beaconInfo.timeTruckStuckMilliSecs );
   printf("Beacon Truck Counter %d\r\n", beaconInfo.newTruckCounter );
   beaconInfo.timeBeaconActive = 5000UL;
   beaconInfo.timeTruckStuckMilliSecs = 8000UL;
   printf("<New parameters>\r\n");
   #endif

   #ifdef BEACON_PRINT
   printf("Beacon On Time %d mSecs\r\n", beaconInfo.timeBeaconActive );
   printf("Beacon Stuck Time %d mSecs\r\n", beaconInfo.timeTruckStuckMilliSecs );
   printf("Beacon Truck Counter %d\r\n", beaconInfo.newTruckCounter );
   #endif

}

/*
 * These are called from commands.c when sensor sends updates to parameters
 * Puts commands
 */
void saveStatusSensorTriggerDistance( uint8_t feetLow, uint8_t feetHigh )
{
   statusSensorTriggerDistanceLow = feetLow;
   statusSensorTriggerDistanceHigh = feetHigh;
}

void saveSensorQualTime( uint32_t time )
{
   statusSensorQualTime = time;
}

void saveSensorQualPostTime( uint32_t time )
{
   statusSensorQualPostTime = time;
}

void saveSensorOperatingStatus( uint16_t status )
{
   sensorOperatingStatus = status;
}


/*
 * Get functions for parameters
 */
uint8_t getSensorTriggerDistanceLow( void )
{
   return( statusSensorTriggerDistanceLow );
}

uint8_t getSensorTriggerDistanceHigh( void )
{
   return( statusSensorTriggerDistanceHigh );
}

uint32_t getSensorQualTime( void )
{
   return( statusSensorQualTime );
}

uint32_t getSensorQualPostTime( void )
{
   return( statusSensorQualPostTime );
}

void clearSensorOperatingStatusRxBit( void )
{
   sensorOperatingStatus &= ~( B_SENSOR_STATUS_NEW );
}

uint16_t getSensorOperatingStatus( void )
{
   return( sensorOperatingStatus );
}

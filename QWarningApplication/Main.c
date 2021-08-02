#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <LPC23xx.H>                                        // LPC213x definitions
#include "shareddefs.h"
#include "sharedInterface.h"
#include "PLL.h"
#include "gpio.h"
#include "irqs.h"
#include "serial.h"
#include "timer.h"
#include "sensor.h"
#include "radar.h"
//#include "Beacon.h"
#include "watchdog.h"
#include "misc.h"
//#include "commands.h"
#include "adc.h"
#include "fram.h"
#include "assetTracker.h"
#include "rateLimiter.h"
#include "log.h"
#ifdef USING_XBEE
#include "serialWireless.h"
#include "wirelessPairing.h"
#include "wireless.h"
#include "cmdProcess.h" //TEMP for ping list and TEST's
#endif

//#define FRAM_TEST
//#define ASSET_TRACKER_TEST


#ifdef ASSET_TRACKER_TEST
void assetTest( uint8_t test ); //TEMP
#endif   // ASSET_TRACKER_TEST




// Forward declarations
static void quaterSecondDoWork( ePLATFORM_TYPE ePlatformType );
static void doRotarySwitch( uint8_t rotarySwitch );
#ifdef FRAM_TEST
static void framTest( void );
#endif
#ifdef JEFFS_SERIAL_DEBUG
static void anyIncomingDebug( void );
#endif

static uint32_t resetRegister;      // Saves the reason for a reset
   
int main (void)
{
   uint32_t k;
   uint32_t quaterSecondTimeStamp;
   ePLATFORM_TYPE ePlatform;


   for( k = 0; k < 100000; k++ ) { __nop( ); }

   // Set up all pins, may be modified in individual init's
   GPIO_CONFIG( );

   // Determine which board we are, save for later
   ePlatform = ePLATFORM_TYPE_QWARNING_SENSOR;

   // Setup clocks, Sensor CPUCLK = 35.0208MHz  Beacon board at 70.016MHz
   PLL_CLOCK_SETUP( ePlatform );
   //do { FIO1SET = ( 1 << 17 ); FIO1CLR = ( 1 << 17 ); } while( 1 );

   // Set up interrupts
   init_VIC( );

   // Set up timers
   timerInit( ePlatform );

   // Set up serial ports, Debug=uart0, radio=uart1,  asset/modem=usrt2, radar=uart3
   serialInit( ePlatform );

   // Non-volatile
   framInit( ePlatform );

   initRadar( );

   // Ultra-sonic sensor
   sensorInit( ePlatform );

   // Get radio into...
   #ifdef USING_XBEE
   wirelessInit( ePlatform );
   #endif

   ADCInit( ePlatform );

   #ifdef BUTTON_PRESENT
   buttonInit( TRUE );
   #endif

// Watchdog 25 seconds
#define WATCHDOG_TIMEOUT_TIME_MS 25000	// timeout in 25 seconds   
// MJB   
watchdogInit((WATCHDOG_TIMEOUT_TIME_MS*8)+1);
   // Watchdog set for 2 seconds
//   watchdogInit( 2000UL );

   // Set up all external LEDS, dry contacts, etc.
   miscInit( );

   assetTrackerInit( );

   #ifdef USING_XBEE
   commandInit( ePlatform );
   #endif

   // Set up x second timer
   quaterSecondTimeStamp = getTimeNow( );

   printf("\r\n    QUEUE WARNING\r\n" );

   printf( "Board Rev %d\r\n", getBoardRev( ) );

   k = SOFTWARE_VERSION;
   printf( "App F/W %d.", k/10000 );
   k -= 10000*(k/10000);
   printf("%02d.", k/100);
   k-= 100*(k/100);
   printf("%02d\r\n", k);

   #if 1
   resetRegister = RSIR;
   printf( "Reset: " );
   if( resetRegister & ( 1 << 0 ) ) { printf( "POR, "); }
   if( resetRegister & ( 1 << 1 ) ) { printf( "External Pin, "); }
   if( resetRegister & ( 1 << 2 ) ) { printf( "WatchDog, "); }
   if( resetRegister & ( 1 << 3 ) ) { printf( "BrownOut "); }
   printf("\r\n");
   #endif

   #ifdef USING_XBEE
   k = wirelessSoftwareVersion( ); x
   printf( "Wireless F/W %d.%d.%d\r\n", BYTE2(k), BYTE1(k), BYTE0(k) );
   #endif

   while( TRUE )
   {
      sensorDoWork( );

      radarSerialDoWork( );

      assetSerialDoWork( );

      #ifdef USING_XBEE
      wirelessSerialDoWork( );

      commandDoWork( );
      #endif

      ledsDoWork( );

      #ifdef BUTTON_PRESENT
      buttonDoWork( );
      #endif

      rateLimiterDoWork( );

      assetTrackerDoWork( );

      radarDoWork( );

      if( hasTimedOut( quaterSecondTimeStamp, 250UL ) )
      {
         quaterSecondTimeStamp = getTimeNow( );
         quaterSecondDoWork( ePlatform );
      }
   }
}
/*
 * This is called every 1/4 second.  Used for non-time
 * critical functions.
 */
static void quaterSecondDoWork( ePLATFORM_TYPE ePlatformType )
{
   static BOOL doneOnce = FALSE;
   static uint8_t counter = 0;

   watchdogFeed( );   // Heavy petting of the dog

   #ifdef USING_XBEE
   wirelessPairingDoWork( );
   #endif

   ADCDoWork( );

   // Only goes through once
   if( FALSE == doneOnce )
   {
      doneOnce = TRUE;
   }

   // Check for special modes
   doRotarySwitch( getModeSelect( ) );

   #ifdef JEFFS_SERIAL_DEBUG
   anyIncomingDebug( );
   #endif

   logDoWork( );

   counter++;

   #ifdef USING_XBEE
   // Poll sensor
   if( ePlatformType == ePLATFORM_TYPE_BEACON_ACTIVATOR )
   {
      static eCOMMANDS normalQueryCmds[] = {
         eCOMMAND_GET_PRESENT_DISTANCE,
         eCOMMAND_WIRELESS_RSSI,
         eCOMMAND_STATUS_BATTERY_VOLTAGE,
         eCOMMAND_GET_SENSOR_PARAMETERS,
      };

      static uint8_t normalCmdIndex = 0;
      static uint32_t pingTimeStamp = 0UL;
      txCommandInfo_t txCommandInfo;

      if( hasTimedOut( pingTimeStamp, COMMAND_TIME_MSECS ) )
      {
         pingTimeStamp = getTimeNow( );
         memset( &txCommandInfo, 0, sizeof( txCommandInfo_t ) );
         txCommandInfo.dataLength = 0;
         txCommandInfo.eCommand = normalQueryCmds[normalCmdIndex++];
         normalCmdIndex = ( sizeof( normalQueryCmds ) / sizeof( normalQueryCmds[0] ) <= normalCmdIndex ) ? 0 : normalCmdIndex;
         sendCommand( &txCommandInfo );
      }

   }

   // Every second, for both devices, check status for LED's
   if( 3 <= counter++ )
   {
      counter = 0;

      // If all ok, turn off yellow led
      if( haveBeenPaired( ) && areDevicesCommunicating( ) )
      {
         setYellowLED( 0 );
      }
      // If lost comms, flash yellow led
      else if( haveBeenPaired( ) && !areDevicesCommunicating( ) )
      {
         toggleYellowLED( );
      }
      // If never paired, solid on
      else
      {
         setYellowLED( 1 );
      }
   }
   #endif   // USING_XBEE

   #if 1
   if( 0 == counter % 20 )    // Every 5 seconds
   {

      #ifdef FRAM_TEST
      framTest( );
      #endif

      #ifdef ASSET_TRACKER_TEST
      assetTest( 2 );
      #endif

      //printf(" Battery voltage %d\r\n", ADCGetBatteryVoltage( ) );
   }
   #endif
}

// Used in factory reset, stays for 500mS then toggles LEDs
static uint32_t showNTell( uint32_t timeStamp )
{
   while( !hasTimedOut( timeStamp, 500UL ) )
   {
      setGreenLED( 0 );
   }
   watchdogFeed( );
   toggleRedLED( );
   toggleYellowLED( );
   setGreenLED( 0 );
   return( getTimeNow( ) );
}

/*** Factory Reset ***/
static void factoryReset( void )
{
   uint32_t timeStamp;
   uint8_t status = 0;

   // Flash LEDs
   setRedLED( 0 );
   setGreenLED( 0 );
   setYellowLED( 1 );
   timeStamp = getTimeNow( );
   timeStamp = showNTell( timeStamp );

   // Erase Fram to 0xFF
   if( !factoryResetFramDevice( ) )
   {
      status |= ( 1 << 0 );
   }
   timeStamp = showNTell( timeStamp );
   timeStamp = showNTell( timeStamp );

   setFactoryDefaults( );
   timeStamp = showNTell( timeStamp );
   timeStamp = showNTell( timeStamp );
   
   #ifdef USING_XBEE
   // Factory reset radio
   if( !backDoorRadio( ePLATFORM_TYPE_FACTORY_RESET_RADIO ) )
   {
      status |= ( 1 << 1 );
   }
   #endif
   timeStamp = showNTell( timeStamp );
   timeStamp = showNTell( timeStamp );

   if( status )
   {
      setRedLED( 1 );
      setYellowLED( 0 );
      printf("Status 0x%X\r\n", status );
      while( 1 )
      {
         watchdogFeed( );
         setGreenLED( 0 );
      }
   }
   else
   {
      setRedLED( 0 );
      setYellowLED( 1 );
      while( 1 )
      {
         watchdogFeed( );
         setGreenLED( 1 );
      }
   }
}


/*** Check for special mode ***/
static void doRotarySwitch( uint8_t rotarySwitch )
{
   if( ePLATFORM_TYPE_QWARNING_SENSOR == rotarySwitch )
   {
      return;
   }
   else
   {
      printf( "WARNING: Rotary switch at %d\r\n", rotarySwitch );
      toggleYellowLED( );
   }
   
   switch( rotarySwitch )
   {
      case ePLATFORM_TYPE_FACTORY_RESET:
         // Rotary switch set to 9 && Reset button was pressed
         if( resetRegister & ( 1 << 1 ) ) 
         {
            factoryReset( );
         }
         break;
      case ePLATFORM_TYPE_ERASE_LOGGING:
      {
         static BOOL done = FALSE;
         if( !done )
         {
            printf("Logging OFF, Erasing Log\r\n");
            turnOffLogging( );
            eraseLogging( );
            done = TRUE;
         }
      }
      break;
      case ePLATFORM_TYPE_LOGGING_ON:
      {
         static BOOL done = FALSE;
         if( !done )
         {
            printf("Logging ON\r\n");
            turnOnLogging( );
            done = TRUE;
         }
         break;
      }
      case ePLATFORM_TYPE_LOGGING_OFF:
         turnOffLogging( );
         break;
   #ifdef USING_XBEE
      case ePLATFORM_TYPE_FACTORY_RESET_RADIO:
         backDoorRadio( ePLATFORM_TYPE_FACTORY_RESET_RADIO );
         break;
      case ePLATFORM_TYPE_DUMP_RADIO_REGS:
         backDoorRadio( ePLATFORM_TYPE_DUMP_RADIO_REGS );
         break;
   #endif
      default:
         break;
   }

}

#ifdef FRAM_TEST
static void framTest( void )
{
   uint16_t i;
   uint8_t writeData[16] = {0, };
   uint8_t readData[16] = {0, };
   uint32_t address = getTimeNow( ) & 0x3FF0;

   for( i = 0; i < sizeof( writeData ); i++ )
   {
      writeData[i] = (uint8_t)rand( );
   }
   printf("Writing @ 0x%X\r\n", address );
   if( writeFramData( address, writeData, sizeof( writeData ) ) )
   {
      memset( readData, 0, sizeof( readData ) );
      printf("Reading from 0x%X\r\n",address );
      if( readFramData( address, readData, sizeof( readData ) ) )
      {
         if( 0 == memcmp( readData, writeData, sizeof( writeData ) ) )
         {
            #if 1    // Spew date read back
            for( i = 0; i < sizeof( writeData ); i++ )
            {
               printf("0x%X ", readData[i] );
            }
            printf("\r\n");
            #endif
         }
         else  // Bad compare
         {
            printf("FRAM Compare Error\r\n");
         }
      }
      else
      {
         printf("FRAM Read Error\r\n");
      }
   }
   else
   {
      printf("FRAM Write Error\r\n");
   }
}
#endif   // FRAM_TEST

#ifdef ASSET_TRACKER_TEST
void assetTest( uint8_t test )
{
   static uint8_t seqNum = 0;

   switch( test )
   {
      case 0:
      {
         uint8_t assetCommand[] = {0x00, 0x00, COMMAND_GET, 24, 0x00};
         //uint8_t i;

         printf("\r\nAsset sending [COMMAND_GET] [24]\r\n");
         assetCommand[LEN_BYTE] = sizeof( assetCommand ) - 1;
         assetCommand[SEQ_BYTE] = seqNum++;
         //for(i=0;i<sizeof(assetCommand); i++) { printf("0x%X\r\n",assetCommand[i]);}
         sendToAsset( assetCommand );
         break;
      }
      case 1:
      {
         uint8_t assetCommand[] = {0x00, 0x00, COMMAND_SET, PARAM_SET_VALUES, 2, 15, 100,0,0,0,  0x90,0x5F,1,0,
                                   0x30,0x75,0,0, 0x40,0x9C,0,0, 0x00};
         //uint8_t i;

         printf("\r\nAsset sending [COMMAND_SET] [PARAM_SET_VALUES]\r\n");
         assetCommand[LEN_BYTE] = sizeof( assetCommand ) - 1;
         assetCommand[SEQ_BYTE] = seqNum++;
         //for(i=0;i<sizeof(assetCommand); i++) { printf("0x%X\r\n",assetCommand[i]);}
         sendToAsset( assetCommand );
         break;
      }
      case 2:
      {
         uint8_t assetCommand[] = {'H', 'e', 'l', 'l', 'o', '\r', 'n', 0};
         uint8_t i;

         printf("\r\nAsset sending text test {");
         for( i = 0; i < sizeof( assetCommand ); i++ ) { printf("0x%X ",assetCommand[i]);} printf("]\r\n");
         sendToAsset( assetCommand );
         break;
      }



      default:
         break;
   }


}
#endif   // ASSET_TRACKER_TEST

#ifdef JEFFS_SERIAL_DEBUG
static void printMenu( void )
{
   printf( " 0 - Reset all devices\r\n" );

   printf( " D - Enable Debug\r\n\n" );

   printf( " F - Factory reset\r\n\n" );

   printf( " L - Start Logging\r\n" );
   printf( " M - Stop Logging\r\n" );
   printf( " N - Dump Log, 1 - 999K ie. N 12\r\n" );
   printf( " O -   \r\n" );
   printf( " P - Set log sample time in seconds\r\n" );
   printf( " Q - Erase Logging\r\n\n" );

   printf( " S - Status\r\n\n" );

   printf( " T - Tokens\r\n\n" );
   
   printf( " X - Exit Debug\r\n" );
   printf( " ? - This menu\r\n" );
}

static void anyIncomingDebug( void )
{
   static BOOL debugEnabled = FALSE;
   static char charBuff[8] = {NULL, };
   static uint8_t index = 0;

   char parm[4] = {0, };
   char ch;
   char cmd;
   uint8_t i;
   uint8_t j = 0;
   uint16_t cliNumber;

   do
   {
      // Any new characters?
      if( 0 == ( U0LSR & RDR ) )
      {
         break;
         //return;
      }

      // Got new char
      ch = U0RBR & 0x07F;     // utf-7

      // Echo?
      if( debugEnabled )
      {
         printf( "%c", ch );
      }

      // Save char
      charBuff[index++] = ch;
      index &= 0x07;

      // Not a CR? Then save and loop again
      if( ( '\r' != ch ) && ( '\n' != ch ) )
      {
         //   charBuff[index++] = ch;
         // index &= 0x07;
         continue;
      }

      // Got here cause got CR, get previous ch, NOTE: \n printed above if enabled
      if( ( '\r' == ch ) && debugEnabled )
      {
         printf( "\n" );
      }

      ch = charBuff[( index - 2 ) & 0x07];  // Get last character

      // Did we get a 'D' and a CR?
      if( ( ch == 'D' ) || ( ch == 'd' ) )
      {
         debugEnabled = TRUE;
      }

      // Debug enabled?
      if( !debugEnabled )
      {
         return;
      }

      // Search for command
      i = ( index - 2 ) & 0x07;
      do
      {
         // Find end of previous cmd
         if( ( '\r' == charBuff[i] ) || ( '\n' == charBuff[i] ) || ( NULL == charBuff[i] ) )
         {
            i = ( i + 1 ) & 0x07;
            cmd = charBuff[i++];      // Save cmd
            //printf("Cmd %c\r\n", cmd);
            break;
         }
         i = ( i - 1 ) & 0x07;
      } while( TRUE );

      // Now get any parameters, i pointing at char after cmd
      memset( parm, 0, sizeof( parm ) );
      j = 0;
      do
      {
         // Find any max 3 digit cliNumber
         if( isalnum( charBuff[i] ) )
         {
            parm[j++] = charBuff[i];
            if( sizeof( parm ) < j )
            {
               break;
            }
         }

         if( ( '\r' == charBuff[i] ) || ( '\n' == charBuff[i] ) || ( NULL == charBuff[i] ) )
         {
            break;
         }
         i = ( i + 1 ) & 0x07;
      } while( TRUE );

      cliNumber = atoi( parm );
      //printf("%d\r\n", cliNumber );

      // Parse
      switch( cmd )
      {
         case '0':
            break;
         case '1':
            break;
         case '2':
            break;
         case '3':
            break;
         case '4':
            break;
         case '5':
            break;
         case '6':
            break;
         case '7':
            break;
         case '8':
            break;
         case '9':
            break;
         case 'A':
         case 'a':
            break;
         case 'B':
         case 'b':
            break;
         case 'C':
         case 'c':
            break;
         case 'E':
         case 'e':
            break;
         case 'D':
         case 'd':
            printf( "\r\n*** Debug mode Enabled ***\r\n" );
            printMenu( );
            break;
         case 'F':
         case 'f':
            break;
         case 'H':
         case 'h':
            break;
         case 'I':
            break;
         case 'L':
         case 'l':
            printf( "Starting Logging\r\n" );
            turnOnLogging( );
            break;
         case 'M':
         case 'm':
            printf( "Stop Logging\r\n" );
            turnOffLogging( );
            break;
         case 'N':
         case 'n':
            dumpLog( cliNumber * 1024UL );
            break;
         case 'O':
         case 'o':

            break;
         case 'P':
         case 'p':
            break;
         case 'Q':
         case 'q':
            printf( "Erasing Log Data\r\n" );
            eraseLogging( );
            break;
         case 'r':
         case 'R':
            break;
         case 'S':
         case 's':
         {
            uint16_t temp16 = SOFTWARE_VERSION;
            printf( "Firmware Ver: %d.", temp16/10000 ); temp16 -= 10000 * ( temp16 / 10000 ); 
            printf( "%d.", temp16/100 );  temp16 -= 100*( temp16 / 100 ); printf( "%d\r\n", temp16 );
            printf( "Board rev %d\r\n", getBoardRev( ) );
            printf( "Periodic reporting telemetry %d Secs\r\n", getReportingTimeSecs( ) );
            printf( "Averaging window %d Secs\r\n", getAverageWindowTimeSecs( ) );
            printf( "Min speed threshold %d MPH\r\n", getMinSpeedThreshold( ) );
            printf( "Total non-zero Radar readings %d\r\n", getNonZeroRadarReadings( INDEX_FOR_30_SECONDS ) );
            printf( "Total Radar readings %d\r\n", getTotalRadarReadings( INDEX_FOR_30_SECONDS ) );
            printf( "Occupancy %d%%\r\n", getOccupancyPercent( INDEX_FOR_30_SECONDS ) );
            printf( "Average speed %d MPH\r\n", getAvgSpeed( INDEX_FOR_30_SECONDS ) );
            printf( "Last radar reading %d MPH\r\n", getLastRadarReading( ) );
            printf( "\r\n" );
            temp16 = getSensorOccupancy( 1000UL * getAverageWindowTimeSecs( ) );
            printf( "Public ultrasonic occupancy %d / %d\r\n", BYTE1( temp16 ), BYTE0( temp16 ) );
            temp16 = getSensorOccupancy( CHECK_FOR_STOPPED_TRAFFIC_MS );
            printf( "Internal ultrasonic occupancy %d / %d\r\n", BYTE1( temp16 ), BYTE0( temp16 ) );
            printf( "Ultrasonic distance trip low %s\r\n", getDistanceString( getSensorTriggerDistanceLow( ) ) );
            printf( "Ultrasonic distance trip high %s\r\n", getDistanceString( getSensorTriggerDistanceHigh( ) ) );
            printf( "Last ultrasonic reading %s\r\n", getDistanceString( getLasSensortReading( ) ) );
            printf( "Battery %d mV\r\n", ADCGetBatteryVoltage( ) );
            printf( "Alarm bits: %s\r\n", printOutAlarmBits( ) );
            printf( "\r\n" );
            printf( "Logging%s Enabled\r\n", isLoggingEnabled( ) ? " is" : " not" );
            break;        
         }
         case 'T':
         case 't':
            printf( "Remaining Tokens %d\r\n", getTokens( ) );
            break;
         case 'X':
         case 'x':
            debugEnabled = FALSE;
            break;
         case 'Y':
         case 'y':
         case 'Z':
         case 'z':
            break;
         case '?':
            printMenu( );
            break;
         default:
            printf( "Unknown command\r\n" );
            break;
      }
   } while( TRUE );
}
#endif   // JEFFS_SERIAL_DEBUG

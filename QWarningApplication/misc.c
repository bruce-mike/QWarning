/*
 * misc.c - Contains miscellaneous functions, mostly turn on/off LED's
 * and toggling port pins and buttons.
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "LPC23xx.h"                        /* LPC23xx/24xx definitions */
#include "stdio.h"
#include "shareddefs.h"
#include "sharedInterface.h"
#include "misc.h"
#include "timer.h"
//#include "beacon.h"
#include "fram.h"
#include "assetTracker.h"
#include "sensor.h"
#include "radar.h"
#ifdef USING_XBEE
#include "wirelessPairing.h"
#include "serialWireless.h"
#include "cmdProcess.h"
#endif

// For debugging, use PCB yellow LED as external AUX_LED0, red LED = Beacon on/off
#define USE_YELLOW_LED
#define USE_RED_LED

#ifdef BUTTON_PRESENT
static void (*buttonState)(void);
static buttonInfo_t buttonInfo;
#endif
static void (*flashingLedState)(void);
static uint32_t toggleTimeStamp;
static uint32_t toggleTime;
static BOOL dryOut0 = FALSE;

// Forward declarations
#ifdef BUTTON_PRESENT
static void buttonNotPressedState( void );
#endif
static void perodicToggleAuxLED0State( void );

/********* MODE/REV *********/

// Returns 0-9 to match selector
// Assumes rotart switch inputs are consecutive
uint8_t getModeSelect( void )
{
   uint8_t portPins;

   portPins = (uint8_t)( ( FIO1PIN >> uP_MOD_x1 ) & 0x000FUL );
   portPins ^= 0x000F;  // Convert port pins to number
   return( (uint8_t)portPins );
}

uint8_t getBoardRev( void )
{
   uint8_t portPins;

   portPins = (uint8_t)( ( FIO1PIN >> REV0 ) & 0x0007UL );
   return( portPins );
}

#ifdef BUTTON_PRESENT
/********************* BUTTON FUNCTIONS ********************
 *
 * Presently the button has 2 functions; 1.) Start pairing
 * and 2.) show which units are paired.  The button goes
 * through a state machine to de-bounce it, over kill?
 * If pressed and released within X seconds will flash
 * the lights of the 2 units that are paired.
 * If pressed and held for more than Y seconds, then released,
 * it will start the pairing, the other node must also go
 * through the same
 */


// Returns TRUE if button is pressed
static BOOL isButtonPressed( void )
{
   return( !( FIO1PIN & ( 1 << PAIR_BUTTON ) ) );
}

/*
 * Called from the main loop.  Will change the sample time depending
 * if the button is active
 */
void buttonDoWork( void )
{
   if( hasTimedOut( buttonInfo.sampleTimeStamp, buttonInfo.samplePeriod ) )
   {
      buttonInfo.sampleTimeStamp = getTimeNow( );
      buttonState( );
   }
}

/*
 * Got here cause button has been pressed for BUTTON_TIME_0 seconds
 * Wait for 2nd time out, if 2nd timer out occurs start pairing
 */
static void buttonPressedStateTime0( void )
{
   if( isButtonPressed( ) ) // Button still pressed?
   {
      buttonInfo.debounceCounter += ( DEBOUNCE_COUNT <= buttonInfo.debounceCounter ) ? 0 : 1;

      if( hasTimedOut( buttonInfo.buttonPressedTimeStamp, BUTTON_TIME_1 ) )
      {
         setAuxLED0( 1 ); // Solid on
      }
   }
   else // Button released after BUTTON_TIME_0, which means start pairing
   {
      if( 0 == buttonInfo.debounceCounter-- )
      {
         buttonInit( TRUE );

         // Start flashing, forever or until time out or paired
         #ifdef USING_XBEE
         setPeriodicAuxLED0( PAIRING_PERIOD_MS );
         startWirelessPairing( );
         setFramSystemPairedBit( FALSE );
         #endif
      }
   }
}

// Got here after button debounced.  Button is PRESSED.
// Wait for time out if it occurs
static void buttonPressedState( void )
{
   if( isButtonPressed( ) ) // Button still pressed?
   {
      buttonInfo.debounceCounter += ( DEBOUNCE_COUNT <= buttonInfo.debounceCounter ) ? 0 : 1;

      if( hasTimedOut( buttonInfo.buttonPressedTimeStamp, BUTTON_TIME_0 ) )
      {
         buttonState = &buttonPressedStateTime0;
         buttonInfo.buttonPressedTimeStamp = getTimeNow( );
         setPeriodicAuxLED0( 1000UL );
      }
   }
   else   // Button released or noise, debounce
   {
      buttonInfo.debounceCounter -= ( buttonInfo.debounceCounter ) ? 1 : 0;

      if( 0 == buttonInfo.debounceCounter )
      {
         #ifdef USING_XBEE
         // To get here means button released before BUTTON_TIME_0, so send
         // command to flash side marker.  If get reply will flash this unit.
         txCommandInfo_t txCommandInfo;

         memset( &txCommandInfo, 0, sizeof( txCommandInfo_t ) );
         txCommandInfo.eCommand = eCOMMAND_FLASH_BEACONS;
         txCommandInfo.dataLength = 0;
         sendCommand( &txCommandInfo );
         #endif
         buttonInit( TRUE );
      }
   }
}

// Got here because button was pressed.  If still pressed
// wait debounce time, else back to not pressed, start over
static void buttonPressedDebounceState( void )
{
   // Button still pressed?
   if( isButtonPressed( ) )
   {
      // Did meet critera?
      if( DEBOUNCE_COUNT <= buttonInfo.debounceCounter++ )
      {
         buttonInfo.buttonPresentState = PRESSED;
         buttonInfo.buttonPressedTimeStamp = getTimeNow( );
         buttonState = &buttonPressedState;
         setAuxLED0( 1 ); // Solid on
      }
   }
   else // Button was released
   {
      if( 0 == buttonInfo.debounceCounter-- )
      {
         buttonInit( TRUE );
      }
   }
}

// Stay here until button is pressed
static void buttonNotPressedState( void )
{
   if( isButtonPressed( ) )
   {
      buttonInfo.samplePeriod = DEBOUNCE_SAMPLE_TIME; // Change button sample time
      buttonInfo.debounceCounter = 1;
      buttonState = &buttonPressedDebounceState;
      setAuxLED0( 1 );
   }
}

// Initialize button state to not pressed
void buttonInit( BOOL resetAux0 )
{
   buttonState = &buttonNotPressedState;
   buttonInfo.buttonPresentState = NOT_PRESSED;
   buttonInfo.debounceCounter = 0;
   buttonInfo.samplePeriod = BUTTON_SAMPLE_TIME;
   buttonInfo.sampleTimeStamp = getTimeNow( );
   if( resetAux0 )
   {
      setAuxLED0( 0 );
   }
}
#endif   // BUTTON_PRESENT

/*
 * Controls external LEDs flashing/on/off and dry contacts
 */
void ledsDoWork( void )
{
   flashingLedState( );
}

void ledNopState( void )
{
}


/************** DRY CONTACTS FUNCTIONS *************
 *
 * To activate the beacons, the signal DRY_OUT must
 * be floating.  To turn off the beacons the signal
 * line must be pulled to ground.
 */
void setDryOut0( BOOL state )
{


   dryOut0 = state;
   if( state )
   {
      FIO4CLR = ( 1 << DRY_OUT0 );
   }
   else
   {
      FIO4SET = ( 1 << DRY_OUT0 );
   }

   #ifdef USE_RED_LED
   setRedLED( state );
   #endif
}

BOOL getDryOut0( void )
{
   return( dryOut0 );
}

void setDryOut1( uint8_t state )
{
   if( state )
   {
      FIO4CLR = ( 1 << DRY_OUT1 );
   }
   else
   {
      FIO4SET = ( 1 << DRY_OUT1 );
   }
}

/********* AUX LED FUNCTIONS *********/

void setAuxLED0( uint8_t on )
{
   if( on )
   {
      flashingLedState = &ledNopState;
      FIO2SET = ( 1 << AUX_LED0 );
   }
   else
   {
      flashingLedState = &ledNopState;
      FIO2CLR = ( 1 << AUX_LED0 );
   }
   #ifdef USE_YELLOW_LED
   setYellowLED( on );
   #endif
}
static void toggleAuxLED0( void )
{
   if( FIO2PIN & ( 1 << AUX_LED0 ) )
   {
      FIO2CLR = ( 1 << AUX_LED0 );
   }
   else
   {
      FIO2SET = ( 1 << AUX_LED0 );
   }
   #ifdef USE_YELLOW_LED
   toggleYellowLED( );
   #endif
}

static void flashAuxLED0( void )
{
   if( hasTimedOut( toggleTimeStamp, 500UL ) )
   {
      toggleAuxLED0( );
      toggleTimeStamp = getTimeNow( );
      toggleTime--;
      if( 0 == toggleTime )
      {
         setAuxLED0( 0 );
         flashingLedState = &ledNopState;
      }
   }
}
// Will flash the AuxLED0 cycle times then quites
void flashAuxLED0NTimes( uint8_t cycles )
{
   setAuxLED0( 1 );
   toggleTime = cycles | 0x01;         // Used as a counter, make odd, starting on
   toggleTimeStamp = getTimeNow( );
   flashingLedState = &flashAuxLED0;
}
/*
 * Will start to toggle AuxLED0 on/off, till told to stop.
 * Input is the period = 1/2 on 1/2 off.  To stop just call
 * setAuxLED0( 0 )
 */
void setPeriodicAuxLED0( uint32_t period )
{
   if( 2 > period )
   {
      return;
   }
   toggleTimeStamp = getTimeNow( );
   toggleTime = period / 2;
   flashingLedState = &perodicToggleAuxLED0State;
}
static void perodicToggleAuxLED0State( void )
{
   if( hasTimedOut( toggleTimeStamp, toggleTime ) )
   {
      toggleAuxLED0( );
      toggleTimeStamp = getTimeNow( );
   }
}

/********* INTERNAL LED FUNCTIONS *********/

void setGreenLED( uint8_t on )
{
   if( on )
   {
      FIO1CLR = ( 1 << LED_GREEN );
   }
   else
   {
      FIO1SET = ( 1 << LED_GREEN );
   }
}
void setYellowLED( uint8_t on )
{
   if( on )
   {
      FIO1CLR = ( 1 << LED_YELLOW );
   }
   else
   {
      FIO1SET = ( 1 << LED_YELLOW );
   }
}
void setRedLED( uint8_t on )
{
   if( on )
   {
      FIO1CLR = ( 1 << LED_RED );
   }
   else
   {
      FIO1SET = ( 1 << LED_RED );
   }
}
void toggleGreenLED( void )
{
   if( FIO1PIN & ( 1 << LED_GREEN ) )
   {
      FIO1CLR = ( 1 << LED_GREEN );
   }
   else
   {
      FIO1SET = ( 1 << LED_GREEN );
   }
}
void toggleYellowLED( void )
{
   if( FIO1PIN & ( 1 << LED_YELLOW ) )
   {
      FIO1CLR = ( 1 << LED_YELLOW );
   }
   else
   {
      FIO1SET = ( 1 << LED_YELLOW );
   }
}
void toggleRedLED( void )
{
   if( FIO1PIN & ( 1 << LED_RED ) )
   {
      FIO1CLR = ( 1 << LED_RED );
   }
   else
   {
      FIO1SET = ( 1 << LED_RED );
   }
}


/********* ULTRA-SONIC FUNCTIONS *********/
#ifdef SENSOR_REAL_TIME
void setHighUltraSonicTrigger( void )
{
   FIO1SET = ( 1 << TRIG_TX );
}

void setLowUltraSonicTrigger( void )
{
   FIO1CLR = ( 1 << TRIG_TX );
}

void startUpUltraSonicTrigger( void )
{
   uint32_t timeStamp = getTimeNow( );

   setHighUltraSonicTrigger( );
   while( !hasTimedOut( timeStamp, 200UL ) );
   setLowUltraSonicTrigger( );

   timeStamp = getTimeNow( );
   while( !hasTimedOut( timeStamp, 5UL ) );
   //setLowUltraSonicTrigger( );
}
#else
void setHighUltraSonicTrigger( void )
{
}
void setLowUltraSonicTrigger( void )
{
}
void startUpUltraSonicTrigger( void )
{
}

#endif   // SENSOR_REAL_TIME

/********* FRAM MISC FUNCTIONS *********/
void disableWriteProtection( void )
{
   FIO2SET = ( 1 << FRAM_WP );
}
void enableWriteProtection( void )
{
   FIO2CLR = ( 1 << FRAM_WP );
}

void setFramChipSelectHigh( void )
{
   FIO0SET = ( 1 << 6 );
}
void setFramChipSelectLow( void )
{
   FIO0CLR = ( 1 << 6 );
}

// Input is TRUE if paired else FALSE for not paired
void setFramSystemPairedBit( BOOL paired )
{
   uint16_t systemFlags;

   readFramData( SYSTEM_FLAGS_ADDR, (uint8_t *)&systemFlags, sizeof( systemFlags ) );

   writeFramData( SYSTEM_FLAGS_ADDR, (uint8_t *)&systemFlags, sizeof( systemFlags ) );
}

// Returns TRUE if been paired, else FLASE for virgin
BOOL haveBeenPaired( void )
{
   uint16_t systemFlags;

   readFramData( SYSTEM_FLAGS_ADDR, (uint8_t *)&systemFlags, sizeof( systemFlags ) );
   return( !( systemFlags & B_DEVICE_NOT_PAIRED ) );
}

// Set/get logging sample rate
void setFramSystemLoggingTime( uint8_t seconds )
{
   writeFramData( LOGGING_TIME_SECS_ADDR, (uint8_t *)&seconds, sizeof( seconds ) );
}

uint8_t getFramLoggingSampleTime( void )
{
   uint8_t seconds;

   readFramData( LOGGING_TIME_SECS_ADDR, (uint8_t *)&seconds, sizeof( seconds ) );
   return( seconds );
}

// All FRAM/programable variables set to their default.
// NOTE: This does not initialize the RAM variables from the fram.
void setFactoryDefaults( void )
{
   uint8_t distance;
   uint8_t speed;
   uint32_t avgWindowTimeMSecs;
   uint32_t reportPeriodMSecs;
   
   printf( "\r\nSetting FACTORY DEFAULTS\r\n" );
   distance = DEFAULT_QUALIFIER_QTR_FEET_LOW;
   writeFramData( SENSOR_QUAL_DISTANCE_LOW_ADDR, &distance, sizeof( distance ) );

   distance = DEFAULT_QUALIFIER_QTR_FEET_HIGH;
   writeFramData( SENSOR_QUAL_DISTANCE_HIGH_ADDR, &distance, sizeof( distance ) );

   speed = DEFAULT_MAX_ZERO_SPEED;
   writeFramData( MAX_ZERO_SPEED_ADDR, &speed, sizeof( speed ) );

   avgWindowTimeMSecs = DEFAULT_WINDOW_TIME_MS;
   writeFramData( AVG_TIME_MSEC_ADDR, (uint8_t *)&avgWindowTimeMSecs, sizeof( avgWindowTimeMSecs ) );
   
   reportPeriodMSecs = DEFAULT_REPORT_TIME_MS;
   writeFramData( ASSET_REPORT_TIME_MSEC_ADDR, (uint8_t *)&reportPeriodMSecs, sizeof( reportPeriodMSecs ) );
   
   // Read from fram
   distance = 0;
   readFramData( SENSOR_QUAL_DISTANCE_LOW_ADDR, (uint8_t *)&distance, sizeof( distance ) ); 
   printf("Trigger distance low %d.%d feet\r\n", distance >> 2, ( distance & 0x3 ) * 25 );
   
   distance = 0;
   readFramData( SENSOR_QUAL_DISTANCE_HIGH_ADDR, (uint8_t *)&distance, sizeof( distance ) ); 
   printf("Trigger distance high %d.%d feet\r\n", distance >> 2, ( distance & 0x3 ) * 25 );
   
   speed = 0;
   readFramData( MAX_ZERO_SPEED_ADDR, (uint8_t *)&speed, sizeof( speed ) ); 
   printf("Zero velocity %dmph\r\n", speed );
   
   avgWindowTimeMSecs = 0UL;
   readFramData( AVG_TIME_MSEC_ADDR, (uint8_t *)&avgWindowTimeMSecs, sizeof( avgWindowTimeMSecs ) ); 
   printf("Averaging window %d Secs\r\n", (uint16_t)(avgWindowTimeMSecs/1000UL ) );

   reportPeriodMSecs = 0UL;
   readFramData( ASSET_REPORT_TIME_MSEC_ADDR, (uint8_t *)&reportPeriodMSecs, sizeof( reportPeriodMSecs ) ); 
   printf("Asset reporting time %d Secs\r\n", (uint16_t)(reportPeriodMSecs/1000UL ) );
}

/********* MISC HELPER FUNCTIONS *********/

// Used to extract 4 bytes, 32 bits, from byte payload
uint32_t get32FromPacket( uint8_t *packet )
{
   uint32_t result = 0;

   result |= (uint32_t)( packet[0] ) << 0;
   result |= (uint32_t)( packet[1] ) << 8;
   result |= (uint32_t)( packet[2] ) << 16;
   result |= (uint32_t)( packet[3] ) << 24;

   return( result );
}

uint16_t get16FromPacket( uint8_t *packet )
{
   uint16_t result = 0;

   result |= (uint16_t)( packet[0] ) << 0;
   result |= (uint16_t)( packet[1] ) << 8;

   return( result );
}

/*
 * Initialize LEDs, both external and internal, dry contacts
 * and anything else
 */
void miscInit( void )
{
   setYellowLED( 1 );
   setRedLED( 0 );
   setAuxLED0( 0 );
   setDryOut0( 0 );
   setDryOut1( 0 );
   flashingLedState = &ledNopState;
   //srand( time(0) );
}

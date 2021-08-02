/*
 * Modem application.
 */
#include <stdio.h>                                          // standard I/O .h-file
#include <string.h>
#include <LPC23xx.H>                                        // LPC213x definitions
#include "shareddefs.h"
#include "sharedInterface.h"
#include "serial.h"
#include "timer.h"
#include "Sensor.h"
#include "Beacon.h"
//#include "wireless.h"
#include "Modem.h"
#include "adc.h"
#include "misc.h"

static uint8_t modemAlarmBits = 0;
static uint8_t modemStatusBits = 0;


// Forward delclarations
static void modemPacketProcessor( uint8_t *pPacket );
static BOOL validateChecksum( uint8_t *msg );
static void addCheckSum( uint8_t *msg );
static void sendStatusCommand( uint8_t *rxPacket, uint8_t status );
static void sendTelemetry( void );
static void sendSettings( void );
static void interpretSetCommand( uint8_t *packet );

/*
 * This is called from modemSerialDoWork() when the Rx FIFO has
 * some byte(s) in it.  It passes 1 byte at a time to here.
 * This routine will reassemble the raw packet.  It will
 * determine when the packet is complete by looking at the
 * Rx length byte and the # of bytes recieved.  Once complete
 * will call modemPacketProcessor()
 * nData - input character/data
 * *pPacket - pointer to rxModemPacket
 * *pnPacketIndex - pointer to rxModemPacketIndex
 *
 * | Length | Seq #  |  Cmd   | Param  | Data[0] |.........| ChkSum |
 */
void buildRxModemPacket( uint8_t nData, uint8_t *pPacket, int16_t *pnPacketIndex)
{
   static BOOL packetError = FALSE;

   // 1st byte in?
   if( !packetError && ( NO_PACKET_DATA == *pnPacketIndex ) )
   {
      *pnPacketIndex = 0;
   }

   // To get here means normal packet data, save it
   pPacket[*pnPacketIndex] = nData;
   *pnPacketIndex += 1;

   // Check for OV
   if( pPacket[LEN_BYTE] + 1 < *pnPacketIndex )
   {
      printf("\nPACKET ERROR OV %d %d\n\r", pPacket[LEN_BYTE],*pnPacketIndex );
      *pnPacketIndex = NO_PACKET_DATA;
      packetError = TRUE;
      sendStatusCommand( pPacket, STATUS_ERROR_UNKNOWN_CMD );
      return;
   }

   // Check if all bytes recieved
   if( *pnPacketIndex == pPacket[LEN_BYTE] + 1 )
   {
      if( !validateChecksum( pPacket ) )
      {
         printf("\nPACKET ERROR CHKSUM\n\r");
         *pnPacketIndex = NO_PACKET_DATA;
         packetError = TRUE;
         sendStatusCommand( pPacket, STATUS_ERROR_CHECKSUM );
         return;
      }

      // To get here means valid packet.  Now decode command
      modemPacketProcessor( pPacket );

      // Reset variables for next packet
      *pnPacketIndex = NO_PACKET_DATA;
   }
   else
   {
      //printf("[0x%X]\r\n",pPacket[*pnPacketIndex-1]);
   }
}

/*
 * Got here after receiving a valid packet from the modem port.
 * Will parse packet and determine what to do next.
 */
static void modemPacketProcessor( uint8_t *pPacket )
{
   //if(pPacket[CMD_BYTE] == COMMAND_STATUS) { return;} //TEMP Debug loop back
   //if(pPacket[LEN_BYTE] == 'D') { return;} //TEMP Debug loop back

   printf("modemPacketProcessor\r\n");
   //sendStatusCommand( pPacket, STATUS_OKAY );

   switch( pPacket[PARAM_BYTE] )
   {
   case PARAM_TELEMETRY:
   {
      if( COMMAND_GET == pPacket[CMD_BYTE] )
      {
         sendTelemetry( );
      }
      else
      {
         sendStatusCommand( pPacket, STATUS_ERROR_UNKOWN_PARAM );
      }
      break;
   }
   case PARAM_SETTINGS:
   {
      if( COMMAND_GET == pPacket[CMD_BYTE] )
      {
         sendSettings( );
      }
      else
      {
         sendStatusCommand( pPacket, STATUS_ERROR_UNKOWN_PARAM );
      }

      break;
   }
   case PARAM_SET_VALUES:
   {
      if( COMMAND_SET == pPacket[CMD_BYTE] )
      {
         interpretSetCommand( pPacket );
      }
      else
      {
         sendStatusCommand( pPacket, STATUS_ERROR_UNKNOWN_CMD );
      }
      break;
   }
   }
}

static void sendTelemetry( void )
{
   uint8_t telemetryData[26] = {0,};
   uint8_t index;
   uint8_t checksum = 0;
   uint32_t temp;

   telemetryData[0] = 'D';        // start
   telemetryData[1] = '1';        // start
   telemetryData[2] = '3';        // telemetry

   temp = 0x1234;//etBeaconBatteryVoltage( );   // Beacon battery
   telemetryData[3] = BYTE0( temp );
   telemetryData[4] = BYTE1( temp );

   temp = 0x1234;//getSensorBatteryVoltage( );   // Sensor battery
   telemetryData[5] = BYTE0( temp );
   telemetryData[6] = BYTE1( temp );

   telemetryData[7] = 0x12;
   telemetryData[8] = 0;
   telemetryData[9] = 0;

   telemetryData[10] = 0;
   telemetryData[11] = 0;
   telemetryData[12] = 0;
   telemetryData[13] = 0;

   temp = 0;
   telemetryData[14] = BYTE0( temp );       // Truck events
   telemetryData[15] = BYTE1( temp );
   telemetryData[16] = BYTE2( temp );
   telemetryData[17] = BYTE3( temp );

   telemetryData[18] = 0;    // not used
   telemetryData[19] = 0;    // not used
   telemetryData[20] = 0;    // not used
   telemetryData[21] = 0;    // not used
   telemetryData[22] = 0;    // not used
   telemetryData[23] = 0;    // not used
   telemetryData[24] = 0;    // not used

   for( index = 0; index < 25; index++ )
   {
      checksum += telemetryData[index];
   }
   checksum &= 0x000000FF;
   telemetryData[25] = checksum;

   printf("Sending [PARAM_TELEMETRY] to modem\r\n");

   // Load up modem FIFO
   for( index = 0; index < sizeof( telemetryData ); index++ )
   {
      printf("0x%X ",telemetryData[index]);
//      serialSendByteToModem( telemetryData[index] );
   }
   printf("\r\n");
}

static void sendSettings( void )
{
   uint8_t settingsData[26] = {0,};
   uint8_t index;
   uint8_t checksum = 0;
   uint32_t temp = 0;

   settingsData[0] = 'D';        // start
   settingsData[1] = '1';        // start
   settingsData[2] = '4';        // settings

   temp = 010101;             // Software version truck entering
   settingsData[3] = BYTE0( temp );
   settingsData[4] = BYTE1( temp );

   temp = 0x12345678; //wirelessSoftwareVersion( ); // Software radio
   settingsData[5] = BYTE0( temp );
   settingsData[6] = BYTE1( temp );

   settingsData[7] = getBoardRev( );  // Hadware version

   settingsData[8] = getModeSelect( ); // Device type

   settingsData[9] = modemStatusBits;

   settingsData[10] = 0;    // not used
   settingsData[11] = 0;    // not used
   settingsData[12] = 0;    // not used
   settingsData[13] = 0; // not used
   settingsData[14] = 0; // not used
   settingsData[15] = 0;    // not used
   settingsData[16] = 0;    // not used
   settingsData[17] = 0;    // not used
   settingsData[18] = 0;    // not used
   settingsData[19] = 0;    // not used
   settingsData[20] = 0;    // not used
   settingsData[21] = 0;    // not used
   settingsData[22] = 0;    // not used
   settingsData[23] = 0;    // not used
   settingsData[24] = 0;    // not used

   for( index = 0; index < 25; index++ )
   {
      checksum += settingsData[index];
   }
   checksum &= 0x000000FF;
   settingsData[25] = checksum;

   printf("Sending [PARAM_GET_SETTINGS] to modem\r\n");

   for( index = 0; index < sizeof( settingsData ); index++ )
   {
      printf("0x%X ",settingsData[index]);
//      serialSendByteToModem( settingsData[index] );
   }
   printf("\r\n");
}

static void interpretSetCommand( uint8_t *packet )
{
   // Minimum trigger distance, feet
   saveSensorTriggerDistanceLow( packet[DATA_START_BYTE] );

   // Maximum trigger distance, feet
   saveSensorTriggerDistanceHigh( packet[DATA_START_BYTE + 1] );

   sendTelemetry( );
}

void modemSendTruckPresentBeaconOn( void )
{
   modemAlarmBits = ( B_BEACON1_INDICATOR_ON | B_TRUCK_PRESENT );
   sendTelemetry( );

   printf("MODEM: TRUCK PRESENT\r\n");
}

void modemSendBeaconOff( void )
{
   modemAlarmBits &= ~( B_BEACON1_INDICATOR_ON | B_BEACON2_INDICATOR_ON | B_TRUCK_PRESENT | B_TRUCK_STUCK );
   sendTelemetry( );

   printf("MODEM: TRUCK NOT PRESENT\r\n");
}

void modemSendTruckStuckBeaconOn( void )
{
   modemAlarmBits |= ( B_BEACON1_INDICATOR_ON | B_BEACON2_INDICATOR_ON | B_TRUCK_PRESENT | B_TRUCK_STUCK );
   sendTelemetry( );

   printf("MODEM: TRUCK STUCK\r\n");
}

void modemSendLostRadioCommunication( void )
{
   modemAlarmBits |= B_NOT_PAIRED;
   sendTelemetry( );

   printf("MODEM: LOST COMMS\r\n");
}

void modemSendHaveRadioCommunication( void )
{
   modemAlarmBits &= ~B_NOT_PAIRED;
   sendTelemetry( );

   printf("MODEM: GOT COMMS\r\n");
}

/*
 * Will send a command/data to the modem, will add checksum
 * byte to end of passed in array, packetOut.
 */
void sendToModem( uint8_t *packetOut )
{
   uint16_t i;

   addCheckSum( packetOut );

   for( i = 0; i <= packetOut[LEN_BYTE]; i++ )
   {
      printf("0x%X ",packetOut[i]);
//      serialSendByteToModem( packetOut[i] );
   }
   printf("\r\n");
}

static BOOL validateChecksum( uint8_t *msg)
{
   int i;
   uint8_t cSum = 0;

   for( i = 0; i < msg[0]; i++ )
   {
      cSum += msg[i];
   }

   if( cSum == msg[msg[0]] )
   {
      return TRUE;
   }

   return FALSE;
}

// Adds checksum byte to end of incoming array
// Make sure incoming array has room for it.
static void addCheckSum( uint8_t *msg )
{
   int16_t i;
   uint8_t cSum = 0;

   for( i = 0; i < msg[LEN_BYTE]; i++ )
   {
      cSum += msg[i];
   }
   msg[i] = cSum;
}

static void sendStatusCommand( uint8_t *rxPacket, uint8_t status )
{
   uint8_t modemOutStatus[5] = {0,};

   printf("Sending COMMAND_STATUS to modem\r\n");
   modemOutStatus[LEN_BYTE] = 4;
   modemOutStatus[SEQ_BYTE] = rxPacket[SEQ_BYTE];
   modemOutStatus[CMD_BYTE] = COMMAND_STATUS;
   modemOutStatus[PARAM_BYTE] = status;

   sendToModem( modemOutStatus );
}


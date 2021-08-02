/*
 * Asset application.
 */
#include <stdio.h>                                          // standard I/O .h-file
#include <string.h>
#include <LPC23xx.H>                                        // LPC213x definitions
#include "shareddefs.h"
#include "sharedInterface.h"
#include "serial.h"
#include "timer.h"
#include "Sensor.h"
#include "adc.h"
#include "misc.h"
#include "assetTracker.h"
#include "fram.h"
#include "rateLimiter.h"
#include <assert.h>
#include "radar.h"
#include "ProductID.h"
#include "iap.h"
#include "crc32.h"
#include "watchdog.h"

#ifdef USING_XBEE
#include "wireless.h"
#include "serialWireless.h"
#include "cmdProcess.h"
#endif

// Local variables
static uint16_t assetAlarmBits = B_MASTER_ALARM;
static uint32_t assetRxTimeStamp;
static uint32_t reportPeriodMSecs = DEFAULT_REPORT_TIME_MS;
static BOOL sentSettingsReportAfterPOR = FALSE;


// Forward delclarations
static void assetPacketProcessor( uint8_t *pPacket );
static BOOL validateChecksum( uint8_t *msg );
static void addCheckSum( uint8_t *msg );
static void sendStatusCommand( uint8_t status );
static void sendTelemetry( void );
static void sendSettings( void );
static void interpertSetParametersCommand( uint8_t *packet );
static void loadAssetParamsFromFram( void );
static void saveReportingTime( uint16_t reportingTimeSecs );
static void doSoftwareUpdate( uint8_t *packet);


//===================================================================
// CRTITCAL!!!
// this must be aligned on a 4 byte address or the IAP upgrade 
// will be broke !!!
//===================================================================
__align(4) static uint8_t blockCodeUpgrade[FLASH_BLOCK_SIZE];
//===================================================================




/*
 * This is called from assetSerialDoWork() when the Rx FIFO has some byte(s)
 * in it.  It passes 1 byte at a time to here.  This routine will reassemble
 * the raw packet.  It will determine when the packet is complete by looking at the
 * Rx length byte and the # of bytes recieved.  Once complete will call
 * assetPacketProcessor()
 *
 * nData - input character/data
 * *pPacket - pointer to rxAssetPacket
 * *pnPacketIndex - pointer to rxAssetPacketIndex
 *
 * | Length | Seq #  |  Cmd   | Param  | Data[0] |.........| ChkSum |
 */
 
 
#define MAX_ASSET_LENGTH    350

static uint8_t assetBuffer[MAX_ASSET_LENGTH];
static uint16_t assetCursor = 0;

// the Asset Tracker cannot ever have the second length byte with a
// value of 0 because it considers it a string terminator - therefore, 
// a 1 is always added to it 
static uint16_t calculateCommandLength(uint8_t *msg)
{
    uint16_t length = (uint16_t)msg[0] + (uint16_t)((msg[1] - 1) * 256); 
    return (length);
}
    
void buildRxAssetPacket( uint8_t nData )
{
   static uint16_t cmdLen;
    
  // Check for time out on packet stream - if so must be start of new command
   if( hasTimedOut( assetRxTimeStamp, ASSET_RX_TIMEOUT_MS ) )
   {
#ifdef DEBUG_UPGRADE
       printf("\nAsset tracker PACKET TIME OUT %d %d\n\r", assetCursor, calculateCommandLength(assetBuffer)); 
#endif       
       assetCursor = LEN_BYTE;
   }
    
   // save time stamp for next character
   assetRxTimeStamp = getTimeNow( );
   
   // To get here means normal packet data, save it   
   assetBuffer[assetCursor] = nData;
#ifdef DEBUG_UPGRADE
   printf("[%d][%02x] ", assetCursor, nData);
#endif
 
   if (assetCursor == LEN_BYTE_256)
   { 
        cmdLen = calculateCommandLength(&assetBuffer[0]);
#ifdef DEBUG_UPGRADE
        printf("1 cmdLen[%d][%d][%d]\r\n", 
                assetBuffer[LEN_BYTE],
                assetBuffer[LEN_BYTE_256],
                ASSET_COMMAND_LENGTH(assetBuffer[LEN_BYTE], assetBuffer[LEN_BYTE_256]));
#endif
   }
   
   if(assetCursor > LEN_BYTE_256)
   {      
        if(assetCursor >= cmdLen) 
        {
#ifdef DEBUG_UPGRADE
            printf("\r\n2 CMD[%d][%d]\r\n", assetBuffer[CMD_BYTE], assetBuffer[PARAM_BYTE]);
#endif           
            // Reset cursor for next incoming packet
            assetCursor = LEN_BYTE;
        
            // chaeck for valid checksum
            if( validateChecksum( &assetBuffer[0] ) )
            {
                if(assetBuffer[CMD_BYTE] == COMMAND_SOFTWATE_UPDATE)
                {
                    doSoftwareUpdate( assetBuffer );
#ifdef DEBUG_UPGRADE
                    printf("3 Software update command\r\n");
#endif
                }
                else
                {
                    assetPacketProcessor( assetBuffer );
#ifdef DEBUG_UPGRADE
                    printf("3 Standard command\r\n");
#endif
                }
            }
            else
            {
                sendStatusCommand( STATUS_ERROR_CHECKSUM );
#ifdef DEBUG_UPGRADE
                printf("2 Checksum error\r\n");
#endif
            }
        }
    }
    
    if(++assetCursor >= MAX_ASSET_LENGTH)
    {
        assetCursor = LEN_BYTE;
    }
}


/*
 * Got here after receiving a valid packet from the asset port.
 * Will parse packet and determine what to do next.
 */
static void assetPacketProcessor( uint8_t *pPacket )
{
   switch( pPacket[PARAM_BYTE] )
   {
      case PARAM_GET_TELEMETRY:
      {
         if( COMMAND_GET == pPacket[CMD_BYTE] )
         {
            sendTelemetry( );
         }
         else
         {
            sendStatusCommand( STATUS_ERROR_UNKOWN_PARAM );
         }
         break;
      }
      case PARAM_GET_SETTINGS:
      {
         if( COMMAND_GET == pPacket[CMD_BYTE] )
         {
            sendSettings( );
         }
         else
         {
            sendStatusCommand( STATUS_ERROR_UNKOWN_PARAM );
         }
         break;
      }
      case PARAM_SET_VALUES:
      {
         if( COMMAND_SET == pPacket[CMD_BYTE] )
         {
            interpertSetParametersCommand( pPacket );
         }
         else
         {
            sendStatusCommand( STATUS_ERROR_UNKNOWN_CMD );
         }
         break;
      }
      case PARAM_SET_RESET_EVENT_COUNT:
      {
         if( COMMAND_SET == pPacket[CMD_BYTE] )
         {

         }
         else
         {
            sendStatusCommand( STATUS_ERROR_UNKNOWN_CMD );
         }
         break;
      }
      case PARAM_SET_BEACONS:
      {
         if( COMMAND_SET == pPacket[CMD_BYTE] )
         {

         }
         else
         {
            
         }
         break;
      }
      case PARAM_REBOOT:
      {
          if( COMMAND_ACTION == pPacket[CMD_BYTE] )
          {
              sendStatusCommand(STATUS_OKAY);
              
              watchdogReboot();
          }
          else
          {
             sendStatusCommand( STATUS_ERROR_UNKNOWN_CMD );
          }
          break;          
      }
      
            
                  
      default:
         printf("UNKNOWN Asset Tracker command 0x%X 0x%X\r\n", pPacket[PARAM_BYTE], pPacket[CMD_BYTE] );
         break;
   }
}


//#define MJB_NO_TELEMETRY 1

#ifdef MJB_NO_TELEMETRY

static void sendTelemetry( void )
{
    
}

#else

static void sendTelemetry( void )
{
   uint8_t telemetryData[26] = {0,};
   uint8_t index;
   uint8_t checksum = 0;
   uint16_t temp16;
   

   memset( telemetryData, 0, 25 );

   telemetryData[0] = 'D';             // Start for QWS per Mike's email 5/18/21
   telemetryData[1] = '1';
   telemetryData[2] = '6';

   telemetryData[3] = getAvgSpeed( INDEX_FOR_300_SECONDS );
   telemetryData[4] = getAvgSpeed( INDEX_FOR_270_SECONDS );
   telemetryData[5] = getAvgSpeed( INDEX_FOR_240_SECONDS );
   telemetryData[6] = getAvgSpeed( INDEX_FOR_210_SECONDS );
   telemetryData[7] = getAvgSpeed( INDEX_FOR_180_SECONDS );
   telemetryData[8] = getAvgSpeed( INDEX_FOR_150_SECONDS );
   telemetryData[9] = getAvgSpeed( INDEX_FOR_120_SECONDS );
   telemetryData[10] = getAvgSpeed( INDEX_FOR_90_SECONDS );
   telemetryData[11] = getAvgSpeed( INDEX_FOR_60_SECONDS );
   telemetryData[12] = getAvgSpeed( INDEX_FOR_30_SECONDS );

   temp16 = getSensorOccupancy( 1000UL * getAverageWindowTimeSecs( ) );
   telemetryData[13] = BYTE0( temp16 );         // Total sensor readings
   telemetryData[14] = BYTE1( temp16 );         // Total sensor occupied readings
   
   telemetryData[15] = BYTE0( getTotalRadarReadings( INDEX_FOR_PROGRAMMED_SECONDS ) );
   telemetryData[16] = BYTE0( getNonZeroRadarReadings( INDEX_FOR_PROGRAMMED_SECONDS ) );

   telemetryData[17] = BYTE1( ADCGetBatteryVoltage( ) );
   telemetryData[18] = BYTE0( ADCGetBatteryVoltage( ) );

   telemetryData[19] = BYTE1( assetAlarmBits );
   telemetryData[20] = BYTE0( assetAlarmBits );

   telemetryData[21]  = rateLimiterTokenCount( );

   telemetryData[22] = getLastRadarReading( );
   telemetryData[23] = getLasSensortReading( );  // In qtr feet resolution

   for( index = 0; index < 25; index++ )
   {
      checksum += telemetryData[index];
   }
   checksum &= 0x000000FF;
   telemetryData[25] = checksum;

   // Load up asset FIFO
   for( index = 0; index < sizeof( telemetryData ); index++ )
   {
      printf("0x%X ",telemetryData[index]);
      serialSendByteToAsset( telemetryData[index] );
   }
   printf( "  <%d>\r\n", getTimeNow( ) );
   //printTelemetry( telemetryData );
}

#endif  // MJB_NO_TELEMETRY

/*
 * Got here cause COMMAND_GET with PARAM_GET_SETTINGS:
 */
static void sendSettings( void )
{
   uint8_t settingsData[26] = {0,};
   uint8_t index;
   uint8_t checksum = 0;
   uint32_t temp = 0;

   memset(settingsData, 0, 25);
   settingsData[0] = 'D';                       // Start for QWS per Mike's email 5/18/21
   settingsData[1] = '1';
   settingsData[2] = '7';

   temp = SOFTWARE_VERSION;                     // Software version truck entering
   settingsData[3] = BYTE1( temp );
   settingsData[4] = BYTE0( temp );

   settingsData[5] = getBoardRev( );            // H/W ID
   settingsData[6] = getMinSpeedThreshold( );   // Min radar speed mph

   settingsData[7] = BYTE1( getReportingTimeSecs( ) ); 
   settingsData[8] = BYTE0( getReportingTimeSecs( ) );

   settingsData[9] = getSensorTriggerDistanceLow( );   // Returns 1/4 feet resolution
   settingsData[10] = getSensorTriggerDistanceHigh( );

   settingsData[11] = BYTE0( getAverageWindowTimeSecs( ) );     
   
   for( index = 0; index < 25; index++ )
   {
      checksum += settingsData[index];
   }
   checksum &= 0x000000FF;
   settingsData[25] = checksum;

   for( index = 0; index < sizeof( settingsData ); index++ )
   {
      printf("0x%2X ",settingsData[index]);
      serialSendByteToAsset( settingsData[index] );
   }
   printf("\r\n");
}

/*
 * Got here because got COMMAND_SET with PARAM_SET_VALUES.  Will parse
 * and load the new parameters.  Sensor data also saved Fram.
 * NOTE: All programmable parameters are sent in this one command.
 */
static void interpertSetParametersCommand( uint8_t *packet )
{
   // Min radar speed
   saveRadarZeroSpeed( packet[5] );

   // Reporting period
   saveReportingTime( (uint16_t)( packet[7] << 8 ) | (uint16_t)packet[6] );

   // Ultasonic sensor, new parameters in 1/4 feet resolution
   saveSensorTriggerDistanceLow( packet[8] );
   saveSensorTriggerDistanceHigh( packet[9] );

   // Averaging time
   saveAverageWindowTime( (uint16_t)packet[4] );
   
   // Reply with new settings
   sendSettings( );
}

uint16_t getAssetAlarmBits( void )
{
   return( assetAlarmBits );
}

/*
 * Will send a command/data to the asset, will add checksum
 * byte to end of passed in array, packetOut.
 */
void sendToAsset( uint8_t *packetOut )
{
   uint16_t i;
   uint16_t cmdLength = ASSET_COMMAND_LENGTH(packetOut[LEN_BYTE], packetOut[LEN_BYTE_256]);

   addCheckSum( packetOut );

   for( i = 0; i <= cmdLength; i++ )
   {
      //printf("0x%X ",packetOut[i]);
      serialSendByteToAsset( packetOut[i] );
   }
   //printf("\r\n");
}

static BOOL validateChecksum( uint8_t *msg )
{
   uint16_t i;
   uint8_t cSum = 0;
   uint16_t cmdLen;
    
   cmdLen = ASSET_COMMAND_LENGTH(msg[LEN_BYTE], msg[LEN_BYTE_256]);

    
   for( i = 0; i < cmdLen; i++ )
   {
      cSum += msg[i];
   }

printf("Asset tracker CHECKSUM cmdLen[%d] msg[0x%X] calc[0x%X]\r\n", cmdLen, msg[cmdLen], cSum );
   
   for(i=124; i< 133; i++)
   {
       printf("%d[%d] ", i, msg[i]);
   }
   
   printf("\r\n");
   
   
   if( cSum == msg[cmdLen] )
   {
      return( TRUE );
   }
   else
   {
      return( FALSE );
   }
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

static void sendStatusCommand( uint8_t status )
{
   uint8_t assetOutStatus[5] = {0,};

   printf("Returning COMMAND_STATUS [%d] to Asset Tracker\r\n", status);
   assetOutStatus[LEN_BYTE] = 4;
   assetOutStatus[LEN_BYTE_256] = LENGTH_256_OFFSET;
   assetOutStatus[CMD_BYTE] = COMMAND_STATUS;
   assetOutStatus[PARAM_BYTE] = status;

   sendToAsset( assetOutStatus );
}



static uint32_t convertBytesToUint32(uint8_t *bytes)
{
    uint32_t uint32Val;
    
    uint32Val = ((uint32_t)bytes[0] << 24) +
                ((uint32_t)bytes[1] << 16) +
                ((uint32_t)bytes[2] << 8) + bytes[3];

    return uint32Val;
}


static void doSoftwareUpdate(uint8_t *packet)
{
    uint32_t writeAddress;
    uint32_t verifyCrc32;
    uint16_t cmdLength;
    uint16_t blockNum;
    uint8_t targetProductId;
    uint16_t codeBytes;
    uint16_t i;
    uint8_t  status = STATUS_OKAY;
    static uint32_t codeSize;
    static uint32_t codeCrc32;
    
// MJB    
watchdogFeed(); 

printf("** paramByte[%d] ***\r\n", packet[PARAM_BYTE]);   
    
    switch(packet[PARAM_BYTE])
    {
        case PARAM_START_UPGRADE:
printf("PARAM_START_UPGRADE\r\n");
            // data format 9 total bytes
            // [productId][size(MSB)][size][size][size(LSB)][crc32(MSB)][crc32][crc32][crc32(LSB)]              
            targetProductId = packet[DATA_START_BYTE];
            if(targetProductId != PRODUCT_ID_QWS)
            {
printf("file target ID not QWarning\r\n");
                status = STATUS_ERROR_PRODUCT_ID;
            }
            else
            {
                codeSize = convertBytesToUint32(&packet[DATA_START_BYTE + 1]);
                codeCrc32 = convertBytesToUint32(&packet[DATA_START_BYTE + 5]);
                printf("codeSize[%d]  codeCrc32[%08x]\r\n", codeSize, codeCrc32);
                if(!eraseUpgradeSectors())
                {
                    status = STATUS_ERROR_SECTOR_NOT_BLANK;
                }
            }
            break;
        
        case PARAM_WRITE_BLOCK:
printf("PARAM_WRITE_BLOCK\r\n");
            status = STATUS_ERROR_WRITE_BLOCK;
        
            cmdLength = calculateCommandLength(packet);
            // first 2 data bytes is block number
            // remaining data bytes is the data to write        
            codeBytes = cmdLength - DATA_START_BYTE - 3;
printf("Code bytes[%d]\r\n", codeBytes);
            if(codeBytes <= FLASH_BLOCK_SIZE)
            {
                memset(&blockCodeUpgrade[0], 0xFF, FLASH_BLOCK_SIZE);
                for(i=0; i<codeBytes; i++)
                {
                    blockCodeUpgrade[i] = packet[DATA_START_BYTE+2+i];
                }
                // note: block numbers start at 0
                blockNum = packet[DATA_START_BYTE] + (packet[DATA_START_BYTE+1] * 256);
                writeAddress = UPGRADE_CODE_BASE_ADDRESS + (blockNum * FLASH_BLOCK_SIZE);
                if(TRUE == writeFlashData(writeAddress, &blockCodeUpgrade[0], codeBytes))
                {
                    status = STATUS_OKAY;
                }
            }
            break;

        case PARAM_END_UPGRADE:
            verifyCrc32 = crc32Calculate((uint8_t *)UPGRADE_CODE_BASE_ADDRESS, codeSize);
//MJB
printf("END_UPGRADE Size[%d]  Crc[%x]  calcCrc[%x]\r\n", codeSize, codeCrc32, verifyCrc32);
        
            if(codeCrc32 == verifyCrc32)
            {   
                if(!writeUpgradeFlag(PRODUCT_TARGET_ID, codeSize, codeCrc32))
                {
                    status = STATUS_ERROR_WRITE_FLAG;
                }
            }
            else
            {
                status = STATUS_CODE_CS_ERROR;
            }
            break;
            
        default:
            printf("Unknown software upgrade command PARAMETER[%d]\r\n", packet[PARAM_BYTE]);
        break;
    }   

    sendStatusCommand(status);
}

/*
 * Continuously called from main loop.  
 * Gets latest alarm bits, presentAlarmBits, if they have changed from assetAlarmBits,
 * save.  Check if any critical bits changed, if so send up telemetry data.  Check if 
 * time to send up periodic telemtry.  Check if time has expired for POR send telemetry.
 * Else no change, so do nothing.
 */
void assetTrackerDoWork(void)
{
   static uint32_t periodicTimeStamp = 0UL;      
   uint16_t presentAlarmBits = B_MASTER_ALARM;
   
   // Get new alarm bits from sub-systems
   presentAlarmBits |= ( TRUE == rateLimiterIsActive( ) ) ? B_ALARM_MODEM_DATA_RATE : 0;
   
   presentAlarmBits |= getBatteryOperatingStatus( );
   
   presentAlarmBits |= getRadarOperatingStatus( );
   
   presentAlarmBits |= getSensorOperatingStatus( );
   
   // Any changes in alarm bits?
   if( presentAlarmBits != assetAlarmBits )
   {
      uint16_t changedBits;
      
      changedBits = presentAlarmBits ^ assetAlarmBits;   // Which bits changed 
      //printf("Old assetAlarmBits 0x%X  presentAlarmBits 0x%X  delta 0x%X\r\n", assetAlarmBits, presentAlarmBits, changedBits); 

      changedBits = ( changedBits ^ assetAlarmBits ) & changedBits;      // Which bits are newly asserted

      // Determine if any 'critical' bits just got asserted
      changedBits &= CRITICAL_BITS;
      //printf("New crittical asserts 0x%X\r\n", changedBits );
      
      // Save new alarm bits
      assetAlarmBits = presentAlarmBits;
      if( changedBits )
      {
         sendTelemetry( );
      }
   }
   // Time for periodic report?
   else if( ( 0UL != reportPeriodMSecs ) && hasTimedOut( periodicTimeStamp, reportPeriodMSecs ) ) 
   {
      sendTelemetry( );
      periodicTimeStamp = getTimeNow( );
   }
   // Send settings once, 90 seconds after por
   else if( !sentSettingsReportAfterPOR && ( SEND_SETTINGS_AT_POR_MS < getTimeNow( ) ) )
   {
      sentSettingsReportAfterPOR = TRUE;
      sendSettings( );
   }
   else
   {
      
   }
}

void assetTrackerInit( void )
{
   memset( &assetAlarmBits, 0, sizeof( assetAlarmBits ) );
   loadAssetParamsFromFram( );
   assetAlarmBits |= B_MASTER_ALARM;
}

uint16_t getReportingTimeSecs( void )
{
   return( reportPeriodMSecs / 1000UL );
}

static void loadAssetParamsFromFram( void )
{
   uint8_t temp8[4];

   // Periodic report time
   readFramData( ASSET_REPORT_TIME_MSEC_ADDR, temp8, sizeof( reportPeriodMSecs ) );
   reportPeriodMSecs = ( (uint32_t)temp8[3] << 24 ) | ( (uint32_t)temp8[2] << 16 ) |
                       ( (uint32_t)temp8[1] << 8 )  | (uint32_t)temp8[0];

   if( MAX_REPORT_TIME_MS < reportPeriodMSecs )
   {
      reportPeriodMSecs = DEFAULT_REPORT_TIME_MS;
   }
}

static void saveReportingTime( uint16_t reportingTimeSecs )
{
   uint32_t timeMs = 1000UL * (uint32_t)reportingTimeSecs;

   if( MAX_REPORT_TIME_MS < timeMs )
   {
      timeMs = DEFAULT_REPORT_TIME_MS;
   }
   reportPeriodMSecs = timeMs;

   writeFramData( ASSET_REPORT_TIME_MSEC_ADDR, (uint8_t *)&reportPeriodMSecs, sizeof( reportPeriodMSecs ) );
   printf("Saving asset reporting time %dmS\r\n", reportPeriodMSecs );
}

// Debug
char *printOutAlarmBits( void )
{
   uint16_t status = getAssetAlarmBits( );
   static char rtnString[64];
   uint8_t index;
   
   memset( rtnString, 0, sizeof( rtnString ) );
   
   if( ( B_MASTER_ALARM == status ) || ( 0 == status ) )
   {
      sprintf( rtnString, "No alarm bits set" );
      return( rtnString );
   }
   
   if( B_BATTERY_VOLTAGE_WARNING & status )
   {
      index = strlen( rtnString );
      sprintf( &rtnString[index], "Battery Warning, " );
   }
   if( B_BATTERY_VOLTAGE_LOW & status )
   {
      index = strlen( rtnString );
      sprintf( &rtnString[index], "Battery Low, " );
   }
   if( B_BATTERY_VOLTAGE_DISCONNECT & status )
   {
      index = strlen( rtnString );
      sprintf( &rtnString[index], "Battery Disconnecting, " );
   }
   if( B_SENSOR_BLOCKED & status )
   {
      index = strlen( rtnString );
      sprintf( &rtnString[index], "Sensor blocked, " );
   }
   if( B_SENSOR_TIMED_OUT & status )
   {
      index = strlen( rtnString );
      sprintf( &rtnString[index], "Sensor timed out, " );
   }
   if( B_RADAR_TIMED_OUT & status )
   {
      index = strlen( rtnString );
      sprintf( &rtnString[index], "Radar timed out, " );
   }
   if( B_SENSOR_TRAFFIC_STOPPED & status )
   {
      index = strlen( rtnString );
      sprintf( &rtnString[index], "Traffic stopped, " );
   }
   if( B_ALARM_MODEM_DATA_RATE & status )
   {
      index = strlen( rtnString );
      sprintf( &rtnString[index], "Modem tokens, " );
   }
      if( B_SOLAR_CHARGING & status )
   {
      index = strlen( rtnString );
      sprintf( &rtnString[index], "Solar charging, " );
   }
   
   
   return( rtnString );
}

 


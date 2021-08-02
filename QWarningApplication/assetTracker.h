/*
 * AssetTracker.h
 */
#ifndef __ASSET_TRACKER_H 
#define __ASSET_TRACKER_H


typedef enum
{
   clear =  0,
   set =    1
} eSetClear_t;


#define ASSET_RX_TIMEOUT_MS  50
#define DELAY_SENDING_SETTINGS_MS   5000UL
#define TIMEOUT_WAITING_SETTINGS_MS 20000UL
#define SEND_SETTINGS_AT_POR_MS     90000UL

// Reporting to asset tracker times
#define MAX_REPORT_TIME_MS           (10UL * 60UL * 1000UL)
#define MIN_REPORT_TIME_MS           (0UL * 1000UL)
#define DEFAULT_REPORT_TIME_MS       (60UL * 1000UL)  // 60 seconds

#define ASSET_RX_FIFO_LENGTH        350
#define ASSET_TX_FIFO_LENGTH        100
#define MAX_ASSET_PACKET_LENGTH     350

// per Asset Tracker: the LEN_BYTE_256 cannot ever be 0 - add offset of 1
#define LENGTH_256_OFFSET       1
#define ASSET_COMMAND_LENGTH(x,y)    (uint16_t)(x + (y-LENGTH_256_OFFSET)*256)

#define LEN_BYTE                0
#define LEN_BYTE_256            1
#define CMD_BYTE                2
#define PARAM_BYTE              3
#define DATA_START_BYTE         4

// Commands
#define COMMAND_GET                 1
#define COMMAND_SET                 2
#define COMMAND_HISTORY             3
#define COMMAND_SOFTWATE_UPDATE     4
#define COMMAND_STATUS              5
#define COMMAND_DATA                6
#define COMMAND_ACTION              7


// Defined parameter values
#define PARAM_SET_VALUES	         1
#define PARAM_GET_TELEMETRY         15
#define PARAM_GET_SETTINGS          16
#define PARAM_GET_MAINTENANCE_DATA  16
#define PARAM_SET_RESET_EVENT_COUNT 17
#define PARAM_SET_BEACONS	         18

// Parameters for code upgrade - it's okay
// if they are duplicated elsewhere because
// they are specific to the COMMAND_SOFTWATE_UPDATE
#define PARAM_START_UPGRADE         17
#define PARAM_ERASE_UPGRADE_SECTORS 18
#define PARAM_WRITE_BLOCK           19
#define PARAM_END_UPGRADE           20
#define PARAM_REBOOT                21


#define STATUS_OKAY                 0
#define STATUS_ERROR_CHECKSUM       1
#define STATUS_ERROR_UNKNOWN_CMD    2
#define STATUS_ERROR_UNKOWN_PARAM   3

#define STATUS_ERROR_ILLEGAL_SETTING    4
#define STATUS_ERROR_SECTOR_NOT_BLANK   5
#define STATUS_ERROR_WRITE_BLOCK        6
#define STATUS_CODE_CS_ERROR            7
#define STATUS_ERROR_WRITE_FLAG         8
#define STATUS_ERROR_PRODUCT_ID         9


// Alarm bits 
#define B_BATTERY_VOLTAGE_LOW          (1 << 0)    // 1 = Battery < 11.6V
#define B_BATTERY_VOLTAGE_DISCONNECT   (1 << 1)    // 1 = Battery critical, shutting down
#define B_SENSOR_BLOCKED               (1 << 2)    // 1 = Sensor continously reading below minimum distance
#define B_SENSOR_TIMED_OUT             (1 << 3)    // 1 = Sensor timed out, broken
#define B_RADAR_TIMED_OUT              (1 << 4)    // 1 = No readings from radar, broken
#define B_ALARM_MODEM_DATA_RATE        (1 << 5)    // 1 = Rate limiter data rate exceeded

#define B_BATTERY_VOLTAGE_WARNING      (1 << 8)    // 1 = Battery < 12.0V
#define B_SOFTWARE_CHECK_SUM_ERROR     (1 << 9)    // 1 - Download binary had check sum error
#define B_REBOOTING_NOW                (1 << 10)   // 1 - Push asyn with set, then reboot
#define B_SENSOR_TRAFFIC_STOPPED       (1 << 11)   // 1 = Traffic stopped
#define B_SOLAR_CHARGING               (1 << 12)   // 1 = Charging source is solar
#define B_NEW_CODE                     (1 << 13)   //
#define B_UPGRADINGG_CODE_IN_PROGRESS  (1 << 14)   //
#define B_MASTER_ALARM                 (1 << 15)

#define CRITICAL_BITS                  (B_BATTERY_VOLTAGE_LOW | B_BATTERY_VOLTAGE_DISCONNECT | B_SENSOR_BLOCKED | \
                                       B_SENSOR_TIMED_OUT | B_RADAR_TIMED_OUT |B_ALARM_MODEM_DATA_RATE)
                                       
//define(:CriticalRateLimited,          0x0020) # 0000 0000 0010 0000 -- Modem data rate exceeded (note: will not clear for 5 minutes)


extern void buildRxAssetPacket( uint8_t nData );
extern void sendToAsset( uint8_t *theCommand );
extern void assetSendLowBattery( void );
extern void assetSendBatteryGood( void );
extern void assetSendSensorWorking( uint16_t statusBits );
extern void assetSendSensorBroken( uint16_t statusBits );
extern void assetSendManualOverrideChanged( void );

extern void assetTrackerInit( void );
extern void assetTrackerDoWork(void);

extern uint16_t getAssetAlarmBits( void );
extern uint16_t getReportingTimeSecs( void );

extern char *printOutAlarmBits( void );


#endif /* end __ASSET_TRSCKER_H */

/*****************************************************************************
**                            End Of File
******************************************************************************/

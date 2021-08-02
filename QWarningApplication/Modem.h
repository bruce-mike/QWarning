/*
 * Modem.h
 */
#ifndef __MODEM_H 
#define __MODEM_H

#define MODEM_FIFO_LENGTH	10
#define MAX_MODEM_PACKET_LENGTH  100

#define LEN_BYTE                0
#define SEQ_BYTE                1
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
#define PARAM_SET_VALUES	                1
#define PARAM_PRESENT_DISTANCE_FEET       3       // Get byte
#define PARAM_BEACON_RSSI                 4       // Get signed byte
#define PARAM_SENSOR_RSSI                 5       // Get signed byte
#define PARAM_ALARM_BITMASK               6       // Get byte
#define PARAM_STATUS_BITMASK              7       // Get byte
#define PARAM_BEACON_BATTERY_VOLTAGE      8       // Get uin16_t
#define PARAM_SENSOR_BATTERY_VOLTAGE      10      // Get uin16_t
#define PARAM_TRUCK_EVENTS                12      // Get uin16_t
#define PARAM_QUALIFING_MIN_DISTANCE_FEET 14      // Get/Set byte qualifing distance threshold
#define PARAM_QUALIFING_MAX_DISTANCE_FEET 15      // Get/Set byte qualifing distance threshold
#define PARAM_QUALIFING_TIME              16      // Get/Set uin16_t qualifing time for distance trigger, 0.1 seconds
#define PARAM_PARKED_TX_TIME              18      // Get/Set uin16_t time out to signal stuck, 0.1 seconds 
#define PARAM_HARDWARE_VERSION            20      // Get byte
#define PARAM_SOFTWARE_VERSION            21      // Get uint16_t
#define PARAM_MODEL_TYPE                  23      // Get byte
#define PARAM_TELEMETRY                   24      // Get
#define PARAM_SETTINGS                    25

#define STATUS_OKAY                 0
#define STATUS_ERROR_CHECKSUM       1
#define STATUS_ERROR_UNKNOWN_CMD    2
#define STATUS_ERROR_UNKOWN_PARAM   3


// Alarm bits for PARAM_ALARM_BITMASK
#define B_BEACON1_INDICATOR_ON      (1 << 0)  // 1=On
#define B_BEACON2_INDICATOR_ON      (1 << 1)  // 1=On
#define B_TRUCK_PRESENT             (1 << 2)  // 1=Sensor detecting truck
#define B_TRUCK_STUCK               (1 << 3)  // 1=Object in front of sensor for X seconds
#define B_NOT_PAIRED                (1 << 4)  // 1=Not paired
#define B_SENSOR_VOLTAGE            (1 << 5)  // 1=UV
#define B_BEACON_VOLTAGE            (1 << 6)  // 1=UV

// Staus bits for PARAM_STATUS_BITMASK
#define B_BEACON1_INDICATOR_ON        (1 << 0)  // 1=On
#define B_BEACON2_INDICATOR_ON        (1 << 1)  // 1=On
#define B_TRUCK_PRESENT               (1 << 2)  // 1=Sensor detecting truck
#define B_TRUCK_STUCK                 (1 << 3)  // 1=Object in front of sensor for X seconds


extern void buildRxModemPacket( uint8_t nData, uint8_t *pPacket, int16_t *pnPacketIndex);
extern void sendToModem( uint8_t *theCommand );
extern void modemSendTruckPresentBeaconOn( void );
extern void modemSendTruckStuckBeaconOn( void );
extern void modemSendBeaconOff( void );
extern void modemSendLostRadioCommunication( void );
extern void modemSendHaveRadioCommunication( void );

#endif /* end __MODEM_H */

/*****************************************************************************
**                            End Of File
******************************************************************************/

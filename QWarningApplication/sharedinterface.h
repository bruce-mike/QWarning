#ifndef SHARED_INTERFACE_H
#define SHARED_INTERFACE_H


#ifndef PACKED
#define PACKED __packed
#endif


#define SOFTWARE_VERSION 10301
//#define SOFTWARE_VERSION 50403


/////////////////////////////////////////////////
// Packet info
// Over the air packet lay out
// 
// |  0x7E  |  Len   |  Seq   |  Cmd  | ....Payload.... | CRC lsb | CRC msb |
// |<------------------------  MAX_PACKET_LENGTH  ------------------------->|
//                                    
/////////////////////////////////////////////////

#define MAX_PACKET_LENGTH 20
#define PACKET_OVERHEAD         6
#define MAX_PACKET_PAYLOAD_LENGTH (MAX_PACKET_LENGTH-PACKET_OVERHEAD)

/////////////////////////////////////////////////
// platform type
/////////////////////////////////////////////////
// Platform type
typedef enum ePlatformType
{
   ePLATFORM_TYPE_QWARNING_SENSOR = 0,
	ePLATFORM_TYPE_BEACON_ACTIVATOR = 1,
   /* Below for debug only */
   ePLATFORM_TYPE_ERASE_LOGGING = 2,
   ePLATFORM_TYPE_LOGGING_ON = 3,
   ePLATFORM_TYPE_LOGGING_OFF = 4,
   
   ePLATFORM_TYPE_DUMP_RADIO_REGS = 6,
   ePLATFORM_TYPE_FACTORY_RESET_RADIO = 7,
   ePLATFORM_TYPE_FACTORY_RESET = 9
} ePLATFORM_TYPE;


/////////////////////////////////////////////////
// interface definitions
/////////////////////////////////////////////////
typedef enum eInterface
{
	eINTERFACE_DEBUG			   = 0,
	eINTERFACE_WIRELESS		   = 1,
   eINTERFACE_ASSET_TRACKER   = 2,
   eINTERFACE_RADAR			   = 3,
	eINTERFACE_MODEM			   = 4,
	eINTERFACE_UNKNOWN		   = 4
}eINTERFACE;

#define INTERFACE_COUNT	((int)eINTERFACE_UNKNOWN)
#define COMMAND_TIME_MSECS    2500UL   // Time to xmit a packet from beacon
/*
 * Differnt type of commands
 * 1.) Command(s) - will get a REPLY, (Has data), or just a plain ACK, (No data)  
 * 2.) Async Status/Alarm
 */
typedef enum eCommands
{
   eCOMMAND_PING                                = 0x00,
   eCOMMAND_ACK                                 = 0x01,
   eCOMMAND_REPLY                               = 0x02,

   eCOMMAND_WIRELESS_CONFIG                     = 0x0A,
   eCOMMAND_WIRELESS_WANCO                      = 0x0B,
   eCOMMAND_WIRELESS_RSSI                       = 0x0C,
  
   eCOMMAND_GET_MODEL_TYPE                      = 0x10,
   eCOMMAND_STATUS_BATTERY_VOLTAGE              = 0x11,
  
   eCOMMAND_ASYNC_TRUCK_PRESENT                 = 0x50,  /* Sensor -> Beacon only */
   eCOMMAND_ASYNC_TRUCK_NOT_PRESENT             = 0x51,  /* Sensor -> Beacon only */  
   eCOMMAND_ASYNC_TRUCK_BLOCKED                 = 0x54,  /* Sensor -> Beacon only */
   eCOMMAND_GET_PRESENT_DISTANCE                = 0x55,  /* Sensor -> Beacon only */
   eCOMMAND_GET_EVENT_COUNT                     = 0x56,  /* Sensor -> Beacon only */
   eCOMMAND_GET_SENSOR_PARAMETERS               = 0x57,  /* Sensor -> Beacon only */
  
   eCOMMAND_SET_TRIGGER_DISTANCE_HIGH           = 0x60,  /* Beacon -> Sensor only */
   eCOMMAND_SET_TRIGGER_DISTANCE_LOW            = 0x61,  /* Beacon -> Sensor only */
   eCOMMAND_SET_TRIGGER_QUALIFYING_TIME         = 0x62,  /* Beacon -> Sensor only */ 

   eCOMMAND_SET_CONTINUOUS_TRIGGER_TIMEOUT      = 0x64,  /* Modem -> Beacon only */ 
   eCOMMAND_SET_CONTINUOUS_TX_PERIOD            = 0x65,  /* Modem -> Beacon only */
   eCOMMAND_SET_TRIGGER_QUALIFYING_POST_TIME    = 0x66,  /* Beacon -> Sensor only */ 
  
  
   eCOMMAND_FLASH_BEACONS                       = 0x70
      
 
} eCOMMANDS;


/*****  FRAM ADDRESSES  *****/

#define SENSOR_QUAL_DISTANCE_LOW_ADDR           0x20       // Byte
#define SENSOR_QUAL_DISTANCE_HIGH_ADDR          0x21       // Byte
#define SYSTEM_FLAGS_ADDR                       0x22       // 2 bytes
#define LOGGING_TIME_SECS_ADDR                  0x24       // Byte
#define MAX_ZERO_SPEED_ADDR                     0x25       // Byte
#define AVG_TIME_MSEC_ADDR                      0x26       // 4 bytes
#define ASSET_REPORT_TIME_MSEC_ADDR             0x2A       // 4 bytes


#define LOGGING_POINTER                         0x50        // 4 bytes
#define LOGGING_START_ADDR                      0x0000100UL // Must be on 0x100 boundry
#define LOGGING_END_ADDR                        0x1000000UL // 128Kbits = 16MBytes on 0x100 boundry

// Fram system flags
#define B_DEVICE_NOT_PAIRED                     (1 << 0)
#define B_LOGGIN_NOT_ENABLED                    (1 << 1)

#endif		// SHARED_INTERFACE_H

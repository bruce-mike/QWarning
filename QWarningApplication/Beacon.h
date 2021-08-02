/*
 * Beacon.h
 */
#ifndef __BEACON_H 
#define __BEACON_H

#define MIN_BEACON_ON_TIME             1000UL // 15000UL
#define DEFAULT_BEACON_ON_TIME         90000UL
#define MAX_BEACON_ON_TIME             255000UL //  250000UL

#define MIN_TRUCK_STUCK_TIME           10000UL
#define DEFAULT_TRUCK_STUCK_TIME       30000UL
#define MAX_TRUCK_STUCK_TIME           60000UL // 250000UL

typedef enum
{
   eManualOverRideOn    =  1,
   eOn                  =  2,
   eSensorActivated     =  3,
   eOff                 =  4
} eBeaconMode_t;

typedef struct
{
  uint32_t timeStampTruckEntered;
  uint32_t timeStampTruckExited;
  uint32_t timeBeaconActive;
  uint32_t newTruckCounter;
  uint32_t timeStampTruckStuck;
  uint32_t timeTruckStuckMilliSecs;
  uint8_t truckEnteringActive : 1;     // Set when receive truck present from sensor
  uint8_t truckExitingActive  : 1;     // Set when receive no truck from sensor
  uint8_t truckStuck          : 1;     // Set when exceeded timeTruckStuckMilliSecs
  uint8_t lostRadioComms      : 1;     // Set when not paired
  uint8_t lowBeaconVoltage    : 1;     // Set when battery is low
  uint8_t lowSensorVoltage    : 1;     // Set when battery is low
  uint8_t badSensor           : 1;     // Set when sensor not reading
  uint8_t blockedSensor       : 1;
  ePLATFORM_TYPE ePlatform;
} beaconInfo_t;

extern void beaconInit( ePLATFORM_TYPE ePlatformType );
extern void beaconDoWork( void );

extern void saveLatestSensorDistance( uint32_t distance );
extern uint32_t getSensorDistance( void );
extern void gotRxTruckPresent( uint32_t distance );
extern void gotRxTruckNotPresent( uint32_t distance );
extern void resetEventCount( void );
extern void setBeaconMode( eBeaconMode_t mode );
extern void saveSensorRssi( int8_t rssi );
extern int8_t getSensorRssi( void );
extern int8_t getBeaconRssi( void );

extern void saveSensorBatterVoltage( uint16_t voltage );
extern uint16_t getSensorBatteryVoltage( void );
extern uint16_t getBeaconBatteryVoltage( void );
extern uint32_t getTruckEventCounter( void );

extern void saveBeaconTruckStuckTime( uint32_t mSecs );
extern void saveBeaconTxTruckStuckPeriod( uint32_t mSecs );
extern void saveBeaconOnTime( uint32_t mSecs );

extern void saveStatusSensorTriggerDistance( uint8_t feetLow, uint8_t feetHigh );
extern void saveSensorQualTime( uint32_t time );
extern void saveSensorQualPostTime( uint32_t time );
extern uint8_t getSensorTriggerDistanceLow( void );
extern uint8_t getSensorTriggerDistanceHigh( void );
extern uint32_t getSensorQualTime( void );
extern uint32_t getSensorQualPostTime( void );

extern uint32_t getBeaconTruckStuckTime( void );
extern uint32_t getBeaconOnTime( void );
extern eBeaconMode_t getBeaconMode( void );
extern uint16_t getSensorOperatingStatus( void );
extern void clearSensorOperatingStatusRxBit( void );


#endif /* end __BEACON_H */

/*****************************************************************************
**                            End Of File
******************************************************************************/

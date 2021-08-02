/*
 * Sensor.h
 */
#ifndef __SENSOR_H 
#define __SENSOR_H

#define TRIG_TX_PIN   8       // P1.8
#define ECHO_RX_PIN   17      // P0.17

// Sensor mesurement ouput conversion
#define TICKS_PER_1MSEC                8755UL   // For timer 1, 
#define SENSOR_COUNT_TO_QTR_FEET       25137UL  // (3.28ft/1M)(.001M/1uS)(.1142uS/tick)(4qtr_ft/1 ft)*2^24  
#define Q_SHIFT_COUNT_TO_QTR_FEET      24       // Q24 number

// Low qualified distances
#define MIN_QUALIFIER_QTR_FEET_LOW     6UL      // (1.5ft) The present sensor, 7363, has a dynamic range 
                                                // of roughly 1.5 to 32 feet.  Anything closer than 1.5 feet will be
                                                // reported as 1.5 feet.  Anything longer than 32 feet will be reported as 32 feet.

#define DEFAULT_QUALIFIER_QTR_FEET_LOW (6UL*4UL) // (6.0ft) Objects must be above this amount
#define MAX_QUALIFIER_QTR_FEET_LOW     (129UL)  // (32.25ft)  This is the max 

// High qualified distances
#define MIN_QUALIFIER_QTR_FEET_HIGH    (7UL)    // (1.75ft)
#define DEFAULT_QUALIFIER_QTR_FEET_HIGH (130UL) // (32.5ft) Objects must be below this amount
#define MAX_QUALIFIER_QTR_FEET_HIGH    (130UL)  // (32.5ft)

#define NO_SENSOR_READING_MSEC         5000UL   // Time out if no sensor reading
#define SENSOR_BLOCKED_TIME_MSEC       60000UL  // Time that we read min distance and set blocked fault

//#define SENSOR_UPDATE_TIME_MSEC        (2100UL) // How often new start new reading, mSecs//
#define SENSOR_UPDATE_TIME_MSEC        (1850UL) // How often new start new reading, mSecs
#define N_SENSOR_READINGS              128      // Must be power of 2^n. 128 * 2.1secs = 269 seconds record
#define MASK_N_SENSOR_READINGS         (0x007F)

#define SHIFT_N_SENSOR_READINGS        5        // Used to get average
#define BACK_5_SECONDS_INDEX           11       // Go back 5 seconds in index's, must be odd!
#define THIRTY_SECS_DTATA_LEN          64       // Ideally should be entire saved data length

#define N_SENSOR_READINGS_STOPPED      (5UL)    // How many samples to check to determine if traffic stopped
#define CHECK_FOR_STOPPED_TRAFFIC_MS   ((N_SENSOR_READINGS_STOPPED * SENSOR_UPDATE_TIME_MSEC) + 100UL)

typedef struct
{
  uint32_t timeStampSensorLastReading;
  uint32_t timeStampSensorBlocked;
  uint32_t timeStampCheckStoppedTraffic;
  uint8_t sensorTriggerDistanceLow;    // 1/4 Foot
  uint8_t sensorTriggerDistanceHigh;   // 1/4 Foot
  uint8_t distanceQtrFeet[N_SENSOR_READINGS];
  BOOL objectPresent[N_SENSOR_READINGS];
  uint8_t distanceIndex;
  uint8_t percentOccupied;
  ePLATFORM_TYPE ePlatform;
  volatile uint8_t pulseWidthReady     : 1;
  volatile uint8_t sensorTimedOutErr   : 1;
  volatile uint8_t sensorBlockedErr    : 1;
  volatile uint8_t sensorBlockedFlag   : 1;
  volatile uint8_t sensorIsTriggered   : 1;
   volatile uint8_t trafficStopped     : 1;
  volatile uint8_t unused              : 2;
} sensorInfo_t;

extern void sensorInit( ePLATFORM_TYPE ePlatformType );
extern void sensorDoWork( void );
extern void saveSensorTriggerDistanceLow( uint8_t distanceFeet );
extern void saveSensorTriggerDistanceHigh( uint8_t distanceFeet );

extern uint8_t getSensorTriggerDistanceLow(  void );
extern uint8_t getSensorTriggerDistanceHigh( void );

extern uint16_t getSensorOperatingStatus( void );
extern uint16_t getAverageDistance( void );
extern uint16_t getSensorOccupancy( uint32_t timeMs );
extern char *getDistanceString( uint8_t distance );
extern uint8_t getLasSensortReading( void );


#endif /* end __SENSOR_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/

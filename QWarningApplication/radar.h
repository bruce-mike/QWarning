/*
 * radar.h
 */
#ifndef __RADAR_H 
#define __RADAR_H

/* MPH */
#define MAX_ZERO_SPEED           (10)     
#define MIN_ZERO_SPEED           (1)     
#define DEFAULT_MAX_ZERO_SPEED   (3)

/* Averaging window */
#define MAX_WINDOW_TIME_MS       (600UL*1000UL)    
#define MIN_WINDOW_TIME_MS       (3UL*1000UL)
#define DEFAULT_WINDOW_TIME_MS   (60UL*1000UL)


#define RADAR_FIFO_LENGTH        (24)     /* Serial FIFO */
#define RADAR_PACKET_LENGTH      (7)

#define INDEX_FOR_PROGRAMMED_SECONDS   0
#define INDEX_FOR_300_SECONDS          1
#define INDEX_FOR_270_SECONDS          2
#define INDEX_FOR_240_SECONDS          3
#define INDEX_FOR_210_SECONDS          4
#define INDEX_FOR_180_SECONDS          5
#define INDEX_FOR_150_SECONDS          6
#define INDEX_FOR_120_SECONDS          7 
#define INDEX_FOR_90_SECONDS           8
#define INDEX_FOR_60_SECONDS           9
#define INDEX_FOR_30_SECONDS           10
#define INDEX_FOR_10_SECONDS           11
#define RADAR_SAVED_DATA_LEN           12


#define RADAR_PACKET_TIMEOUT_MS  (6000UL)    // 6 seconds

#define RADAR_SPEED_STOPPED      (8)      /* Used for stopped bit */
#define RADAR_SPEED_MOVING       (16)

#define RADAR_SAMPLE_RATE        (366UL)  /* 2^8/0.7 seconds = 1.43 samples/seconds * 2^8 */
#define Q_RADAR_SAMPLE_RATE      (8)

#define RADAR_RAW_DATA_LEN       (512)    /* Must be power of 2^n */
#define MASK_RAW_RADAR_DATA      0x01FF

#define TIME_300_SECONDS         (300UL*1000UL)
#define TIME_270_SECONDS         (270UL*1000UL)
#define TIME_240_SECONDS         (240UL*1000UL)
#define TIME_210_SECONDS         (210UL*1000UL)
#define TIME_180_SECONDS         (180UL*1000UL)
#define TIME_150_SECONDS         (150UL*1000UL)
#define TIME_120_SECONDS         (120UL*1000UL)
#define TIME_90_SECONDS          (90UL*1000UL)
#define TIME_60_SECONDS          (60UL*1000UL)
#define TIME_30_SECONDS          (30UL*1000UL)
#define TIME_10_SECONDS          (10UL*1000UL)


typedef PACKED struct
{
   uint8_t avgSpeed;
   uint8_t occupancyPercent;
   uint16_t totalSpeedCount;
   uint16_t nonZeroSpeedCount;
} calcInfo_t;

typedef struct
{
   uint8_t rawSpeed[RADAR_RAW_DATA_LEN];           // All speed readings, including 0
   uint16_t rawSpeedIndex;
   calcInfo_t lastRadarInfo[RADAR_SAVED_DATA_LEN]; // Array of last avg speed @ differnt time windows 
   uint32_t timeStampWindow[RADAR_SAVED_DATA_LEN]; // Array of last time a time windows info was calculated
   uint8_t maxZeroSpeed;                           // Below this is considered 0 mph
   uint32_t windowAveragingTime;                   // Time span to do averaging
   uint8_t readingTimedOut          :1;
   uint8_t readingTimedOutSet       :1;
   uint8_t generateAllInfo          :1;
   uint8_t unUsed                   :5;
} radarInfo_t;

extern void initRadar( void );
extern void radarDoWork( void );
extern void buildRxRadarPacket( uint8_t nData, uint8_t *pPacket, int16_t *pnPacketIndex );

extern void saveRadarZeroSpeed( uint8_t speed );
extern uint8_t getMinSpeedThreshold( void );
extern void saveAverageWindowTime( uint16_t avgWindowTimeSecs );
extern uint16_t getAverageWindowTimeSecs( void );
extern uint8_t getAvgSpeed( uint8_t timeIndex );
extern uint8_t getTotalRadarReadings( uint8_t timeIndex );
extern uint8_t getNonZeroRadarReadings( uint8_t timeIndex );
extern uint8_t getOccupancyPercent( uint8_t timeIndex );
extern uint8_t getLastRadarReading( void );
extern uint16_t getRadarOperatingStatus( void );

#endif   // __RADAR_H

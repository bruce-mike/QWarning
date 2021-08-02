#ifndef MISC_H
#define MISC_H

#define PAIR_BUTTON           4
#define PAIRED_FLASH_CYCLES   4
#define BUTTON_SAMPLE_TIME    250UL
#define DEBOUNCE_SAMPLE_TIME  25UL   // mSecs
#define DEBOUNCE_COUNT        5
#define BUTTON_TIME_0         4000UL
#define BUTTON_TIME_1         5000UL
#define PAIRING_PERIOD_MS     4000UL

typedef enum
{
  NOT_PRESSED,
  PRESSED
} buttonPresentState_t;

typedef struct
{
  uint32_t sampleTimeStamp;   // mSecs
  uint32_t samplePeriod;      // mSecs
  uint32_t buttonPressedTimeStamp;
  uint8_t debounceCounter;    // Counts down if not pressed.  Counts up if pressed
  buttonPresentState_t buttonPresentState;
} buttonInfo_t;

extern void buttonDoWork( void );
extern void buttonInit( BOOL resetAux0 );

extern void setGreenLED( uint8_t on );
extern void setYellowLED( uint8_t on );
extern void setRedLED( uint8_t on );
extern void toggleGreenLED( void );
extern void toggleYellowLED( void );
extern void toggleRedLED( void );
extern uint8_t getModeSelect( void );
extern uint8_t getBoardRev( void );

extern void setAuxLED0( uint8_t on );
extern void setDryOut0( uint8_t on );
extern BOOL getDryOut0( void );
extern void setDryOut1( uint8_t on );

extern void miscInit( void );
extern void ledsDoWork( void );
extern void setPeriodicAuxLED0( uint32_t period );
extern void flashAuxLED0NTimes( uint8_t cycles );

// Sensor
extern void setHighUltraSonicTrigger( void );
extern void setLowUltraSonicTrigger( void );
extern void startUpUltraSonicTrigger( void );

// FRAM
extern void disableWriteProtection( void );
extern void enableWriteProtection( void );
extern void setFramChipSelectHigh( void );
extern void setFramChipSelectLow( void );
extern void setFactoryDefaults( void );

// Radio
extern void setFramSystemPairedBit( BOOL pairedBit );
extern BOOL haveBeenPaired( void );

// Logging
extern void setFramSystemLoggingTime( uint8_t seconds );
extern uint8_t getFramLoggingSampleTime( void );

extern uint32_t get32FromPacket( uint8_t *packet );
extern uint16_t get16FromPacket( uint8_t *packet );


#endif		// MISC_H

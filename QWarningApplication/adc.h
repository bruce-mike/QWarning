/*****************************************************************************
 *   adc.h:  Header file for NXP LPC23xx Family Microprocessors
 *
 *   Copyright(C) 2006, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2006.09.20  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#ifndef __ADC_H 
#define __ADC_H

typedef struct
{
  uint8_t chan0 : 1;
  uint8_t chan1 : 1;
  uint8_t chan2 : 1;
  uint8_t chan3 : 1;
  uint8_t chan4 : 1;
  uint8_t chan5 : 1;
  uint8_t chan6 : 1;
  uint8_t irqOccurred : 1;    // Must change if channel 7 is used
} adcDone_t;

typedef struct 
{
   uint8_t batteryVoltageWarning :1;
   uint8_t batteryVoltageLow     :1;
   uint8_t batteryVoltageCritical:1;
   uint8_t unUsed                :5;
} batteryInfo_t;

#define ADC_INTERRUPT_FLAG	1	/* 1 is interrupt driven, 0 is polling */

#define ADC_OFFSET		0x10
#define ADC_INDEX			4

#define ADC_DONE			0x80000000
#define ADC_OVERRUN		0x40000000
#define ADC_ADINT			0x00010000

#define ADC_MAX_CHANS	3					/* Channels 0, 1 & 2 */
#define ADC_CLK				1000000		/* set to 1Mhz */

#define ADVREF				(3.0)
#define VS_REF        (3.3) 	// REFERENCE FOR 'VBAT MONITOR' DIFF AMP 
#define NUMBITS				(10)	 	// conversion 'width' N bits (aka 10 bits /8 bits)


static const float ADC_SCALE_FACTOR = (ADVREF/(1 << NUMBITS));
#define VBATT_SLOPE     4751        // .01856 * 1000 * 2^8 Get in mV & Q8
#define VBATT_B         37094       // .1449 * 1000 * 2^8
#define VBATT_Q         8

#define ADC_BATTERY_INIT   720      // Fill in array so averaging doesn't take long time

void ADCInit( ePLATFORM_TYPE ePlatformType );
void ADCDoWork(void);
uint16_t ADCGetRssiVoltage( void );
uint16_t ADCGetBatteryVoltage( void );
uint16_t getBatteryOperatingStatus( void );

#endif /* end __ADC_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/

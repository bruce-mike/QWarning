/*****************************************************************************
 *   adc.c:  ADC module file for NXP LPC23xx Family Microprocessors
 *
 *   Copyright(C) 2006, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2006.08.15  ver 1.00    Prelimnary version, first Release
 *
 ******************************************************************************/
#include "LPC23xx.h"                        /* LPC23xx definitions */
#include <stdio.h>
#include <string.h>
#include "shareddefs.h"
#include "sharedInterface.h"
#include "timer.h"
#include "adc.h"
#include "irqs.h"
#include "assetTracker.h"

/*
 * This file has been modified to support the Truck Entering
 * project.  It uses 2 A/D inputs; AD0.0 and AD0.1, will use
 * AD0.2 for the future, i.e. will convert the input but presently
 * not used.
 * AD0.0 = Battery voltage
 * AD0.1 = RSSI
 * AD0.2 = Converted, not used.
 */
#define ADC_CHANNEL_RSSI_VOLTAGE    1
#define ADC_CHANNEL_BATTERY_VOLTAGE 2

#define ADC_Fpclk ( 36000000 / 4 )    /* Clk into ADC */
#define ADC_SAMPLE_TIME_MS  ( 1000UL / ADC_MAX_CHANS )  /* Each channel sampled at 1 second */
//#define ADC_SAMPLE_TIME_MS  1000UL

// Local variables
static volatile uint32_t ADC0Value[ADC_MAX_CHANS];
static volatile adcDone_t adcDone;
static uint16_t arrayChan0[4];
static uint16_t arrayChan1[4];
static uint16_t arrayChan2[4];
static TIMERCTL adcSampleTimer;
static batteryInfo_t batteryInfo = {0, };



#if ADC_INTERRUPT_FLAG
/******************************************************************************
** Function name:		ADC0Handler
**
** Descriptions:		ADC0 interrupt handler
**
** parameters:			None
** Returned value:		None
**
******************************************************************************/
static void ADC0Handler (void) __irq
{
   uint32_t regVal;

   regVal = AD0STAT;          /* Read ADC will clear the interrupt */

   if( regVal & 0x0000FF00 )  /* check OVERRUN error first */
   {
      printf("ADC Overrun\n");

      regVal = ( regVal & 0x0000FF00 ) >> 0x08;
      /* if overrun, just read ADDR to clear */
      /* regVal variable has been reused. */
      switch( regVal )
      {
         case ( 1 << 0 ):
            regVal = AD0DR0;
            break;

         case ( 1 << 1 ):
            regVal = AD0DR1;
            break;

         case ( 1 << 2 ):
            regVal = AD0DR2;
            break;

         default:
            break;
      }
   }

   /* Read ADC conversions */
   if( regVal & ADC_ADINT )
   {
      switch( regVal & 0xFF )             /* Check individual DONE bit(s) */
      {
         case ( 1 << 0 ):
            ADC0Value[0] = ( AD0DR0 >> 6 ) & 0x3FF;
            adcDone.chan0 = 1;
            break;

         case ( 1 << 1 ):
            ADC0Value[1] = ( AD0DR1 >> 6 ) & 0x3FF;
            adcDone.chan1 = 1;
            break;

         case ( 1 << 2 ):
            ADC0Value[2] = ( AD0DR2 >> 6 ) & 0x3FF;
            adcDone.chan2 = 1;
            break;

         default:
            break;
      }
   }
   adcDone.irqOccurred = 1;       // Semaphore

   AD0CR &= 0xF8FFFFFF; /* stop ADC now */
   VICVectAddr = 0;           /* Acknowledge Interrupt */
}
#endif

/*****************************************************************************
** Function name:		ADCHWInit
**
** Descriptions:		initialize ADC channel
**
** parameters:			ADC clock rate
** Returned value:		true or false
**
*****************************************************************************/

static uint32_t ADCHWInit( uint32_t ADC_Clk )
{
   /* Enable CLOCK into ADC controller */
   PCONP |= ( 1 << 12 );

   //Pin select done in gpio.c

   AD0CR = ( 0x01 << 0 ) |    /* SEL=1,select channel 0~7 on ADC0 */
           ( ( ADC_Fpclk / ADC_Clk - 1 ) << 8 ) |   /* CLKDIV = ADC_Fpclk / 1000000 - 1 */
           ( 0 << 16 ) |            /* BURST = 0, no BURST, software controlled */
           ( 0 << 17 ) |            /* CLKS = 0, 11 clocks/10 bits */
           ( 1 << 21 ) |            /* PDN = 1, normal operation */
           ( 0 << 22 ) |            /* TEST1:0 = 00 */
           ( 0 << 24 ) |            /* START = 0 A/D conversion stops */
           ( 0 << 27 );                   /* EDGE = 0 (CAP/MAT singal falling,trigger A/D conversion) */


   /* If POLLING, no need to do the following */
   #if ADC_INTERRUPT_FLAG
   AD0INTEN = 0x007;          /* Enable ADC channels 2, 1 & 0 */
   if( install_irq( ADC0_INT, ADC0Handler, HIGHEST_PRIORITY ) == FALSE )
   {
      printf("ADC_INT: Failed to Install!\n");
      return ( FALSE );
   }
   #endif
   return ( TRUE );
}

/*****************************************************************************
** Function name:		ADC0Read
**
** Descriptions:		If ADC_INTERRUPT_FLAG = 0 this will start ADC0 on
**									the passed in channel, then wait for completion of
**									the conversion, returns the ADC data
**
**                            If ADC_INTERRUPT_FLAG = 1 this will start ADC0 on
**									the passed in channel, then return.  Interrupt
**									handler will fill in ADC0Value[X] &	ADCXIntDone
**
** parameters:			Channel number
** Returned value:		Value read, if interrupt driven, return channel #
**
*****************************************************************************/
static uint32_t ADC0Read( uint8_t channelNum )
{
   #if !ADC_INTERRUPT_FLAG
   uint32_t regVal, ADC_Data;
   #endif

   /* Allowable channel number is 0 through 7 */
   if( ADC_MAX_CHANS < channelNum )
   {
      channelNum = 0;               /* reset channel number to 0 */
   }
   AD0CR &= 0xFFFFFF00;                                                             // Clear out channel mask
   AD0CR |= ( 1 << 24 ) | ( 1 << channelNum );  // Select next channel, Start ADC conversion

   #if !ADC_INTERRUPT_FLAG
   while( 1 )                 /* wait until end of A/D convert */
   {
      regVal = *(volatile unsigned long *)( AD0_BASE_ADDR
                                            + ADC_OFFSET + ADC_INDEX * channelNum );
      /* read result of A/D conversion */
      if( regVal & ADC_DONE )
      {
         break;
      }
   }

   AD0CR &= 0xF8FFFFFF; /* stop ADC now */
   if( regVal & ADC_OVERRUN ) /* save data when it's not overrun, otherwise, return zero */
   {
      return ( 0 );
   }

   ADC_Data = ( regVal >> 6 ) & 0x3FF;
   return ( ADC_Data );       /* return A/D conversion value */
   #else
   return ( channelNum );     /* if it's interrupt driven, the ADC reading is
                                          done inside the handler. so, return channel number */
   #endif
}

//============================================
void ADCInit( ePLATFORM_TYPE ePlatformType )
{
   uint8_t i;

   initTimer(&adcSampleTimer);

   memset( (unsigned char*)ADC0Value, 0, sizeof( ADC0Value ) );

   memset( (unsigned char*)&adcDone, 0, sizeof( adcDone_t ) );

   memset( arrayChan1, 0, sizeof( arrayChan1 ) );
   memset( arrayChan2, 0, sizeof( arrayChan2 ) );
   memset( &batteryInfo, 0, sizeof( batteryInfo ) );
   
   // Initialize to nominal batter reading
   for( i = 0; i < sizeof( arrayChan0 ) / sizeof( arrayChan0[0] ); i++ )
   {
      arrayChan0[i] = ADC_BATTERY_INIT;
   }

   ADCHWInit( ADC_CLK );

   startTimer(&adcSampleTimer, ADC_SAMPLE_TIME_MS);
}

/*
 * Called from asset tracker code.  Will return the present alarm bits
 * releated to the battery voltage.
 */
uint16_t getBatteryOperatingStatus( void )
{
   uint16_t status = 0;
   uint16_t presentBatteryVoltage = ADCGetBatteryVoltage( );   // Average of last n readings
   
   status |= ( WARN_BATTERY_VOLTAGE > presentBatteryVoltage ) ? B_BATTERY_VOLTAGE_WARNING : 0;
   status |= ( LOW_BATTERY_VOLTAGE > presentBatteryVoltage ) ? B_BATTERY_VOLTAGE_LOW : 0;
   status |= ( CRITICAL_BATTERY_VOLTAGE > presentBatteryVoltage ) ? B_BATTERY_VOLTAGE_DISCONNECT : 0;

   return( status );
}

/*
 * This is called from the main() loop.
 * Read ADC status register, determine if an ADC conversion is complete
 * If so read it, optionally filter/average and store it.
 * Determine next channel, then start the next conversion.
 */
void ADCDoWork()
{
   static uint8_t nChannelNum = 1;
   static uint8_t index = 0;

   /* read result of A/D conversion */
   if( adcDone.irqOccurred )    // Conversion done?
   {
      adcDone.irqOccurred = 0;

      if( adcDone.chan0 )
      {
         adcDone.chan0 = 0;
         arrayChan0[index] = ADC0Value[0];
         nChannelNum = 1; // Next channel
      }
      else if( adcDone.chan1 )
      {
         adcDone.chan1 = 0;
         arrayChan1[index] = ADC0Value[1];
         nChannelNum = 2; // Next channel
      }
      else if( adcDone.chan2 )
      {
         adcDone.chan2 = 0;
         arrayChan2[index] = ADC0Value[2];
         index++;
         if( sizeof( arrayChan1 ) / sizeof( arrayChan1[0] ) <= index )
         {
            index = 0;    // Next channel
         }
         nChannelNum = 0;
      }
      else
      {
         nChannelNum = 0;
         printf("ADCDoWork: Unknown done\n\r");
      }
   }

   if( isTimerExpired(&adcSampleTimer) )
   {
      ADC0Read(nChannelNum);  // This starts ADC conversion on nChannelNum
      startTimer(&adcSampleTimer, ADC_SAMPLE_TIME_MS);
   }
}

// Assumes max ADC value is 10 bits
static uint16_t getAverageADC( uint16_t *array, uint16_t length )
{
   uint16_t i;
   uint16_t sum = 0;

   for( i = 0; i < length; i++ )
   {
      sum += array[i];
   }
   return( sum / length );
}

uint16_t ADCGetRssiVoltage( void )
{
   return( getAverageADC( arrayChan1, sizeof( arrayChan1 ) / sizeof( arrayChan1[0] ) ) );
}

// Returns in mV
uint16_t ADCGetBatteryVoltage( void )
{
   uint32_t temp = (uint32_t)getAverageADC( arrayChan0, sizeof( arrayChan0 ) / sizeof( arrayChan0[0] ) );

   temp *= VBATT_SLOPE;
   temp += VBATT_B;
   temp >>= VBATT_Q;
   return( (uint16_t)temp );
}


//============================================
/*********************************************************************************
**                            End Of File
*********************************************************************************/


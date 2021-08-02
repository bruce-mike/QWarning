#include <stdlib.h>
#include <stdio.h>
#include "shareddefs.h"
#include "sharedInterface.h"
#include "lpc23xx.h"
#include "irqs.h"
#include "PLL.h"
#include "timer.h"
#include "misc.h"

#include "gpio.h"//TEMP

static ePLATFORM_TYPE ePlatformType;
static int32_t nLEDCycles;
volatile static uint32_t nCurrentMilliseconds;
volatile static uint8_t nCurrentEpoch;


/*
 * Timer0 interrupt. Set up to occur every 1mSecs.  Acts as a free running
 * counter and is the bases for all app timer functions.
 */
void T0Handler(void)  __irq
{
   // acknowledge interrupt
   T0IR = ( 1 << T0IR_MR3 );

   // 32b int msecs = 1,192h till rollover ~ 50days
   if( 0 == ++nCurrentMilliseconds )
   {
      nCurrentEpoch++;
   }
   if( 0 <= nLEDCycles )
   {
      if( 0 >= --nLEDCycles )
      {
         toggleGreenLED();
         nLEDCycles = 500;
      }
   }
   VICVectAddr = 0;           /* Acknowledge Interrupt */
}

void T1Handler (void) __irq
{
   T1IR  =  ( 1 << T1IR_MR3 );  // Clear match 1 interrupt

   VICVectAddr = 0x00000000;    // Dummy write to signal end of interrupt
}

void initTimer(TIMERCTL* pTimer)
{
   pTimer->nTimeoutEpoch = 0;
   pTimer->nTimeoutTime = 0;
}

/*
 * Set up to create timer0 interrupt every N mSecs.  In reality this
 * is always set for 1mSec cause it is used for other s/w timer
 * functions.  Assumes CPU clock is either 72 or 36MHz.
 */
void T0_SETUP_PERIODIC_INT( uint32_t desired_msecs )
{
   // Turn on periph power, redundent?
   PCONP |= ( 1 << PCTIM0 );

   // Set to INT and reset on match, using MR3
   T0MCR |= ( 1 << MR3I ) | ( 1 << MR3R );

   // Set peripherial clock same for timer0 = 36MHz/4 or 72MHz/8
   switch( ePlatformType )
   {
      case ePLATFORM_TYPE_BEACON_ACTIVATOR:
         PCLKSEL0 &= ~( CCLK_OVER_MASK << PCLK_TIMER0 );
         PCLKSEL0 |= ( CCLK_OVER_8 << PCLK_TIMER0 );
         break;
      case ePLATFORM_TYPE_QWARNING_SENSOR:
         PCLKSEL0 &= ~( CCLK_OVER_MASK << PCLK_TIMER0 );
         PCLKSEL0 |= ( CCLK_OVER_4 << PCLK_TIMER0 );
         break;
      default:
         PCLKSEL0 &= ~( CCLK_OVER_MASK << PCLK_TIMER0 );
         PCLKSEL0 |= ( CCLK_OVER_8 << PCLK_TIMER0 );
         break;
   }
   // Load up interrupt click
   T0MR3 = (BASE_CLK_36MHZ)*( desired_msecs ) / ( 4 * 1000 );

   // install handler
   // not sure why void() (void) __irq is necessary.  NXP demo code called for (void(*))?
   install_irq( TIMER0_INT, ( void ( * )(void) __irq )T0Handler, LOWEST_PRIORITY );

   // Clear/reset and then start counter
   T0TCR = ( 1 << T0_RESET );
   __nop( );
   T0TCR = 0;   // Pull out of reset
   T0TCR = ( 1 << T0_ENA );

}

/* Set up timer1 as a free running timer.  Input is 28.55nS, (1/35.0208MHz).
 * This is divided down further, below, 4, to give 114.2nS/tick.
 * This is used to measure pulse width of sensor.
 */
void T1_SETUP_PERIODIC_INT( uint32_t desired_msecs )
{
   // Turn on periph power, redundent?
   PCONP |= ( 1 << PCTIM1 );

   // Set to INT and reset on match, using MR3
   T1MCR |= ( 1 << MR3I ) | ( 1 << MR3R );

   // Set peripherial clock same for timer0 = 36MHz/4 or 72MHz/8
   // 111.117nS
   switch( ePlatformType )
   {
      case ePLATFORM_TYPE_BEACON_ACTIVATOR:
         PCLKSEL0 &= ~( CCLK_OVER_MASK << PCLK_TIMER1 );
         PCLKSEL0 |= ( CCLK_OVER_8 << PCLK_TIMER1 );
         break;
      case ePLATFORM_TYPE_QWARNING_SENSOR:
         PCLKSEL0 &= ~( CCLK_OVER_MASK << PCLK_TIMER1 );
         PCLKSEL0 |= ( CCLK_OVER_4 << PCLK_TIMER1 );
         break;
      default:
         PCLKSEL0 &= ~( CCLK_OVER_MASK << PCLK_TIMER1 );
         PCLKSEL0 |= ( CCLK_OVER_8 << PCLK_TIMER1 );
         break;
   }

   T1MR3 = 0xFFFFFFFFUL;

   // install handler
   // not sure why void() (void) __irq is necessary.  NXP demo code called for (void(*))?
   install_irq( TIMER1_INT, ( void ( * )(void) __irq )T1Handler, LOWEST_PRIORITY );

   // Clear/reset and then start counter
   T1TCR = ( 1 << T1_RESET );
   __nop( );
   T1TCR = 0;   // Pull out of reset
   T1TCR = ( 1 << T1_ENA );
}

void timerShowLEDHeartbeat(void)
{
   nLEDCycles = 500;
}

void timerStopLEDHeartbeat()
{
   nLEDCycles = -1;
}
/*
 * Returns present free running timer in mSecs.
 */
uint32_t getTimeNow( void )
{
   volatile uint8_t nEpoch = nCurrentEpoch;
   volatile uint32_t nMilliseconds = nCurrentMilliseconds;

   if( nEpoch != nCurrentEpoch )
   {
      /////
      // timer rolled over, so grab it again
      /////
      nEpoch = nCurrentEpoch;
   }
   while( nMilliseconds != nCurrentMilliseconds )
   {
      nMilliseconds = nCurrentMilliseconds;
   }

   return nMilliseconds;
}
/*
 * Helper function that assume passed in timeStamp came from
 * getTimeNow() function.
 */
BOOL hasTimedOut( uint32_t timeStamp, uint32_t delay )
{
   return( ( ( getTimeNow( ) - timeStamp ) >= delay ) ? TRUE : FALSE );
}

/*
 * Used for measuring the pulse width of the ultra-sonic
 * sensor.  Need high resolution in time cause output of
 * sensor is 1uS/1mm
 */
// Returns 111.11nS h/w counter
uint32_t getTimer1Count( void )
{
   volatile uint32_t count0;
   volatile uint32_t count1;

   count0 = T1TC;  // Get present count
   count1 = T1TC;

   // Make sure correct reading
   //while( 100 < ABS((int32_t)(count0-count1)) )
   while( 10 < ( count1 - count0 ) )
   {
      count0 = T1TC; // Re-read
      count1 = T1TC;
   }
   return( count1 );
}

void startTimer(TIMERCTL *pTimer, unsigned int nTimeoutMS)
{
   volatile unsigned char nEpoch = nCurrentEpoch;
   volatile unsigned int nMilliseconds = nCurrentMilliseconds;
   if( nEpoch != nCurrentEpoch )
   {
      /////
      // timer rolled over, so grab it again
      /////
      nEpoch = nCurrentEpoch;
      nMilliseconds = nCurrentMilliseconds;
   }
   pTimer->nTimeoutEpoch = nEpoch;
   while( nMilliseconds != nCurrentMilliseconds )
   {
      nMilliseconds = nCurrentMilliseconds;
   }
   pTimer->nTimeoutTime = nMilliseconds;
   if( 0 < nTimeoutMS )
   {

      pTimer->nTimeoutTime += nTimeoutMS;
      if( pTimer->nTimeoutTime < nMilliseconds )
      {
         /////
         // our timer rolled over
         // so increment to next epoch
         /////
         pTimer->nTimeoutEpoch++;
      }
   }
}

BOOL isTimerExpired(TIMERCTL* pTimer)
{
   BOOL bRetVal = FALSE;
   volatile unsigned char nEpoch = nCurrentEpoch;
   volatile unsigned int nMilliseconds = nCurrentMilliseconds;

   if( nEpoch != nCurrentEpoch )
   {
      nEpoch = nCurrentEpoch;
      nMilliseconds = nCurrentMilliseconds;
   }
   if( pTimer->nTimeoutEpoch < nEpoch )
   {
      bRetVal = TRUE;
   }
   else if( ( pTimer->nTimeoutEpoch == nEpoch ) && ( pTimer->nTimeoutTime < nMilliseconds ) )
   {
      bRetVal = TRUE;
   }
   return bRetVal;
}

void stopTimer(TIMERCTL* pTimer)
{
   pTimer->nTimeoutEpoch = 0;
   pTimer->nTimeoutTime = 0;
}

void timerInit( ePLATFORM_TYPE ePlatform )
{
   ePlatformType = ePlatform;
   nCurrentMilliseconds = 0;
   nCurrentEpoch = 0;
   T0_SETUP_PERIODIC_INT(1);
   T1_SETUP_PERIODIC_INT(10);
}
/*****************************************************************************
** Function name:		delayMs
**
** Descriptions:		Start the timer delay in milliseconds
**									until elapsed
**
** parameters:			timer number, Delay value in millisecond
**
** Returned value:		None
**
*****************************************************************************/
void delayMs( uint32_t delayInMs )
{
   TIMERCTL delayTimer;
   initTimer(&delayTimer);
   if( 0 == delayInMs )
   {
      delayInMs = 1;
   }
   startTimer(&delayTimer, delayInMs);
   while( !isTimerExpired(&delayTimer) );
}

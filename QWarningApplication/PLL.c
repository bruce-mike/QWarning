#include "lpc23xx.h"
#include <assert.h>
#include "shareddefs.h"
#include "sharedInterface.h"
#include "PLL.h"


// reconfigure the PLL once we come out of sleep mode(s)
// current usage is for CPU freq = 24MHz
// most PCLKs = 6MHz (cclk/4) (DEFAULT)

static void PLL_CONFIG( ePLATFORM_TYPE ePlatformType )
{

// FROM DATASHEET:
// 1 DISCONNECT PLL WITH FEED
// 2 DISABLE PLL WITH FEED
// 3 CHANGE CPU CLK DIVIDER (W/O PLL)
// 4 WRITE CLK SOURCE SEL REG
// 5 WRITE PLLCFG AND FEED
// 6 ENABLE PLL WITH FEED
// 7 CHANGE CPPU CLK DIVIDER (W/PLL)
// 8 WAIT FOR LOCK
// 9 CONNECT PLL WITH FEED

//************************************

   // NOTE: Fcco must be between 275 - 550 MHz
   // Step 4, select external Xtal, as clock source, 3.6864MHz
   CLKSRCSEL = 0x01;

   // Step 5, write PLLCFG.  (multiplier val)
   // fcco=(2 x M x Fin)/N = (2 X 76 X 3686400)/2 = 280166400
   // PLLCFG = (PLL_MULT << MSEL) | (PLL_DIV_72MHZ << NSEL);
   PLLCFG = ( ( 76 - 1 ) << MSEL ) | ( ( 2 - 1 ) << NSEL );

   // Step 6, Enable PLL with feed
   PLLFEED = 0xAA;
   PLLFEED = 0x55;

   // Step 7, determine what CPU clock rate we want, again
   switch( ePlatformType )
   {
      case ePLATFORM_TYPE_QWARNING_SENSOR:
         CCLKCFG = CCLKSEL_36MHZ;
         break;
      case ePLATFORM_TYPE_BEACON_ACTIVATOR:
         CCLKCFG = CCLKSEL_72MHZ;
         break;
      default:
         CCLKCFG = CCLKSEL_36MHZ;
         break;
   }

   // Step 8, wait for lock
   while( !( PLLSTAT & ( 1 << PLLSTAT_PLOCK ) ) ) { __nop( ); }

   // Step 9 connect with feed
   PLLFEED = 0xAA;
   PLLFEED = 0x55;
}
/*
 * Master clock source can be either;
 * 1) Internal RC Oscillator (IRC) = 4MHz,   Fcco = 288MHz
 * 2) External crystal at 3.6864MHz,         Fcco = 280.1666MHz
 * Then divide down to either 36MHz/35.0208MHz or 72MHz/70.016MHz, for the CPU clock.
 * The peripheral clocks will be divided down to the same frequency based on the ePLATFORM_TYPE.
 * All UART's PCLK will be 36MHz/35.0208MHz
 * All ADC's PCLK  will be 9MHz/8.7552MHz
 */
void PLL_CLOCK_SETUP( ePLATFORM_TYPE ePlatformType )
{

   switch( ePlatformType )
   {
      case  ePLATFORM_TYPE_QWARNING_SENSOR:
      {
         // Set up UART0, UART1 and ADC
         PCLKSEL0 &= ( ~( CCLK_OVER_MASK << PCLK_UART0 ) |
                       ( CCLK_OVER_MASK << PCLK_UART1 ) |
                       ( CCLK_OVER_MASK << PCLK_SSP1 ) |
                       ( CCLK_OVER_MASK << PCLK_ADC ) );

         PCLKSEL0 |= ( CCLK_OVER_1 << PCLK_UART0 ) | // Debug
                     ( CCLK_OVER_1 << PCLK_UART1 ) | // Xbee
                     ( CCLK_OVER_1 << PCLK_SSP1 ) | // SPI
                     ( CCLK_OVER_4 << PCLK_ADC );

         // Set up UART2 & UART3
         PCLKSEL1 &= ~( ( CCLK_OVER_MASK << PCLK_UART2 ) | ( CCLK_OVER_MASK << PCLK_UART3 ) );
         PCLKSEL1 |= ( CCLK_OVER_1 << PCLK_UART2 ) | ( CCLK_OVER_1 << PCLK_UART3 ); // Asset tracker & Modem
         break;
      }
      case  ePLATFORM_TYPE_BEACON_ACTIVATOR:
      {
         // Set up UART0, UART1 and ADC
         PCLKSEL0 &= ( ~( CCLK_OVER_MASK << PCLK_UART0 ) |
                       ( CCLK_OVER_MASK << PCLK_UART1 ) |
                       ( CCLK_OVER_MASK << PCLK_SSP1 ) |
                       ( CCLK_OVER_MASK << PCLK_ADC ) );

         PCLKSEL0 |= ( CCLK_OVER_2 << PCLK_UART0 ) | // Debug
                     ( CCLK_OVER_2 << PCLK_UART1 ) | // Xbee
                     ( CCLK_OVER_2 << PCLK_SSP1 ) | // SPI
                     ( CCLK_OVER_8 << PCLK_ADC );

         // Set up UART2 & UART3
         PCLKSEL1 &= ~( ( CCLK_OVER_MASK << PCLK_UART2 ) | ( CCLK_OVER_MASK << PCLK_UART3 ) );
         PCLKSEL1 |= ( CCLK_OVER_2 << PCLK_UART2 ) | ( CCLK_OVER_2 << PCLK_UART3 ); // Asset tracker & Modem
         break;
      }
      default:
         // Set up UART0, UART1 and ADC
         PCLKSEL0 &= ( ~( CCLK_OVER_MASK << PCLK_UART0 ) |
                       ( CCLK_OVER_MASK << PCLK_UART1 ) |
                       ( CCLK_OVER_MASK << PCLK_ADC ) );

         PCLKSEL0 |= ( CCLK_OVER_2 << PCLK_UART0 ) | // Debug
                     ( CCLK_OVER_2 << PCLK_UART1 ) | // Xbee
                     ( CCLK_OVER_8 << PCLK_ADC );

         // Set up UART2 & UART3
         PCLKSEL1 &= ~( ( CCLK_OVER_MASK << PCLK_UART2 ) | ( CCLK_OVER_MASK << PCLK_UART3 ) );
         PCLKSEL1 |= ( CCLK_OVER_2 << PCLK_UART2 ) | ( CCLK_OVER_2 << PCLK_UART3 ); // Asset tracker & Modem
         break;
   }

   /////
   // configure the PLL
   /////
   PLL_CONFIG( ePlatformType );
   /////
   // ENERGIZE PERIPHERALS
   /////
   switch( ePlatformType )
   {
      case  ePLATFORM_TYPE_QWARNING_SENSOR:
      case  ePLATFORM_TYPE_BEACON_ACTIVATOR:
         PCONP |= ( 1 << PCUART0 ) |         // Debug uart
                  ( 1 << PCUART1 ) |         // Xbee
                  ( 1 << PCUART2 ) |         // Xbee
                  ( 1 << PCUART3 ) |         // Modem
                  ( 1 << PCAD   ) |          // ADC
                  ( 1 << PCTIM0 ) |          // Timer0
                  ( 1 << PCTIM1 );           // Timer1
         break;

      default:
         break;
   }
}

//============================================================================
// GPIO control
//============================================================================
#include "LPC23xx.h"                        /* LPC23xx/24xx definitions */
#include "stdio.h"
#include "shareddefs.h"
#include "sharedInterface.h"
#include "misc.h"
/*
 * GPIO CONFIG
 * set up I/Os as in/out, set default states
 * peripheral pins are set
 *
 * NB: LAST STEP IS TO DISABLE ETM IN SW
 * it is configured on boot by sampling RTCK.
 * The value written to the config reg is xRTCK.
 * As drawn, this enables ETM and hoses some PORT2 ops
 */

void GPIO_CONFIG( void )
{


   // Set gpio 0 & 1 to high speed & turn on external oscilator
   SCS = 0x21;
   /*
    *   PORT 0
    *
    * P0.0 = uP_AUX_TX     (Alt-out)      (TXD3)   Radar
    * P0.1 = uP_AUX_RX     (Alt-in)       (RXD3)   Radar
    * P0.2 = uP_DBG_TXD    (Alt-out)      (TXD0)   Debug
    * P0.3 = uP_DBG_RXD    (Alt-in)       (RXD0)   Debug
    *
    * P0.6 = FRAM_CS#      (Alt-out)      (SSEL1)  FRAM
    * P0.7 = SCK           (Alt-out)      (SCK1)
    * P0.8 = MISO          (Alt-in)       (MISO1)
    * P0.9 = MOSI          (Alt-out)      (MOSI1)
    *
    * P0.15 = XB_TX        (Alt-out)      (TXD1)   Radio
    *
    * P0.16 = XB_RX        (Alt-in)       (RXD1)   Radio
    * P0.17 = ECHO_IN      (Digital-in)
    * P0.17 = ECHO_RX      (Digital-in-pu)
    *
    * P0.19 = XB_RST       (Digital-out)           Radio
    *
    * P0.22 = XB_RTS       (Alt-out)               Radio
    * P0.23 = uP_ADC_Vb    (Analog-in)
    * P0.24 = RSSI         (Analog-in)
    * P0.25 =              (Analog-in)
    *
    *
    */

   // Pin select 00 = dio, 01 = alt func, 10 = alt func, 11 = alt fuc
   PINSEL0 = ( 1 << ( 15 * 2 ) ) | ( 2 << ( 9 * 2 ) ) | ( 2 << ( 8 * 2 ) ) | ( 2 << ( 7 * 2 ) ) | ( 0 << ( 6 * 2 ) ) |
             ( 1 << ( 3 * 2 ) ) | ( 1 << ( 2 * 2 ) ) | ( 2 << ( 1 * 2 ) ) | ( 2 << ( 0 * 2 ) ); // P0.0 - P0.15
   PINSEL1 = ( 1 << ( 24 - 16 ) * 2 ) | ( 1 << ( 23 - 16 ) * 2 ) | ( 0 << ( 19 - 16 ) * 2 ) |
             ( 1 << ( 25 - 16 ) * 2 ) | ( 1 << ( 16 - 16 ) * 2 ) | ( 0 << ( 27 - 16 ) * 2 ); // P0.16 - P0.31

   // Pin directions
   FIO0DIR = 0UL;
   FIO0DIR = ( OUT << uP_AUX_TX ) | ( IN << uP_AUX_RX ) | ( OUT << uP_DBG_TXD ) | ( IN << uP_DBG_RXD ) |
             ( OUT << FRAM_CS ) | ( OUT << SPI_SCK ) | ( IN << SPI_MISO ) | ( OUT << SPI_MOSI ) |
             ( OUT << XB_TX ) | ( IN << XB_RX ) | ( OUT << XB_RST ) | ( IN << XB_CTS ) | ( OUT << XB_RTS ) |
             ( IN << uP_ADC_Vb ) | ( IN << RSSI ) | ( IN << uP_DEBUG ) | ( IN << ECHO_RX );

   #ifdef USE_CTS
   PINSEL1 |= ( 1 << ( 17 - 16 ) * 2 );
   #endif //USE_CTS
   #ifdef USE_RTS
   PINSEL1 |= ( 1 << ( 22 - 16 ) * 2 );
   #endif //USE_RTS

   // Pin initial values
   FIO0SET = 0xFFFFFFFFUL;
   FIO0CLR = ( ( 1 << XB_RST ) ); // Radio in reset, dry contacts off // MJF
   /*
    *   PORT 1
    *
    * P1.4 = ECHO_IN      ( Input-pu)  NOTE Bit Bank only
    *
    * P1.8 = TRIG_TX      (Digital-out)
    *
    * P1.15 = LED_RED     (Digital-out)
    * P1.16 = LED_YELLOW  (Digital-out)
    * P1.17 = LED_GREEN   (Digital-out)
    *
    * P1.19 = uP_MOD_x1   (Digital-in-float)
    * P1.20 = uP_MOD_x2   (Digital-in-float)
    * P1.21 = uP_MOD_x4   (Digital-in-float)
    * P1.22 = uP_MOD_x8   (Digital-in-float)
    *
    * P1.27 = REV0        (Digital-in-pu)
    * P1.28 = REV1        (Digital-in-pu)
    * P1.29 = REV2        (Digital-in-pu)
    */

   // Pin select 00 = dio, 01 = alt func, 10 = alt func, 11 = alt fuc
   PINSEL2 = 0UL;       // P1.0 - P1.15
   PINSEL3 = 0UL;       // P1.16 - P1.31

   // Pin directions
   FIO1DIR = ( IN << REV0 ) | ( IN << REV1 ) | ( IN << REV2 ) |
             ( IN << uP_MOD_x8 ) | ( IN << uP_MOD_x4 ) | ( IN << uP_MOD_x2 ) | ( IN << uP_MOD_x1 ) |
             ( OUT << LED_GREEN ) | ( OUT << LED_YELLOW ) | ( OUT << LED_RED ) |
             ( OUT << TRIG_TX ) | ( IN << ECHO_IN );

   // Pull-up
   PINMODE3 &= ~( ( 3 << ( 2 * ( REV0 - 16 ) ) ) | ( 3 << ( 2 * ( REV1 - 16 ) ) ) | ( 3 << ( 2 * ( REV2 - 16 ) ) ) );

   // Pin initial values
   FIO1SET = 0xFFFFFFFFUL;

   FIO1CLR = ( 1 << LED_GREEN );
   #ifdef SENSOR_REAL_TIME
   FIO1CLR = ( 1 << TRIG_TX );
   #endif

   /*
    *   PORT 2
    *
    * P2.0 = WP#           (Digital-out)  // FRAM ~WP
    * P2.1 = AUX_LED0      (Digital-out)  // Side marker
    *
    * P2.8 = MODEM_TX      (Alt-out)
    * P2.9 = MODEM_RX      (Alt-in)
    *
    */

   // Pin select 00 = dio, 01 = alt func, 10 = alt func, 11 = alt fuc
   PINSEL4 = ( 2 << ( 9 * 2 ) ) | ( 2 << ( 8 * 2 ) );  // P2.0 - P2.15,

   FIO2DIR = ( OUT << AUX_LED0 ) | ( OUT << FRAM_WP ) | ( OUT << MODEM_TX );

   // Pin initial values
   FIO2SET = 0xFFFFFFFFUL;

   FIO2CLR = ( 1 << AUX_LED0 );


   // // last step of setup - disable ETM
   // // (ETM hoses some PORT2 ops (P2[9:0])
   // // should fix this in HW - change pull
   PINSEL10 = 0x00000000;
   //DISABLE_ETM();

   /*
    *   PORT 3
    * P3.28 = START_ULTRA_SONIC   (Digital-out)
    *
    * !!! NOTE: Initial DENUG Only
    */
   PINSEL7 = 0UL;          // P3.16 - P3.31
   FIO3DIR = ( OUT << START_ULTRA_SONIC );
   #ifdef SENSOR_REAL_TIME
   FIO3CLR = ( 1 << START_ULTRA_SONIC );
   #else
   FIO3SET = ( 1 << START_ULTRA_SONIC );
   #endif
   /*
    *   PORT 4
    *
    * P4.28 = DRY_OUT1    (Digital-out)
    * P4.29 = DRY_OUT0    (Digital-out)
    *
    */
   PINSEL9 = 0UL;          // P4.16 - P4.31
   FIO4DIR = ( OUT << DRY_OUT1 ) | ( OUT << DRY_OUT0 );
   FIO4CLR = ( 1 << DRY_OUT1 ) | ( 1 << DRY_OUT0 );

}


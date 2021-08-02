//========================================================================================
// Shared definitions for all files
//========================================================================================

#ifndef SHARED_DEFS_H
#define SHARED_DEFS_H

//#define JEFF_DEV_MODE          // For testing indoors, (shorter range), etc.
#define JEFFS_SERIAL_DEBUG       // Command line debug
#define SENSOR_REAL_TIME

#define TRUE 1
#define FALSE 0

// DATA DIRECTIONS/pin modes 
#define IN			0
#define OUT			1

typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;
typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef unsigned long long uint64_t;
typedef unsigned char  BOOL;

#define LOW_HIGH_BYTES_TO_16( low, high ) (((uint16_t)(high)<<8) | (uint16_t)(low))

#define BYTE0( x ) (uint8_t)((x) & 0x0FF)
#define BYTE1( x ) (uint8_t)(((x) >> 8) & 0xFF)
#define BYTE2( x ) (uint8_t)(((x) >> 16) & 0xFF)
#define BYTE3( x ) (uint8_t)(((x) >> 24) & 0xFF)

#define ABS( x ) (((x) < 0) ? (-(x)) : (x))
                                       
#define MIN( x, y ) (((x) < (y)) ? (x) : (y))
#define MAX( x, y ) (((x) > (y)) ? (x) : (y))

// PERIPHERAL POWER CONTROL BITS
// 0 UNUSED
#define PCTIM0   1
#define PCTIM1   2
#define PCUART0  3
#define PCUART1  4
// 5 UNUSED
#define PCPWM1   6
#define PCI2C0   7
#define PCSPI    8
#define PCRTC    9
#define PCSSP1  10
#define PCEMC   11
#define PCAD    12
#define PCAN1   13
#define PCAN2   14
// 15 UNUSED
// 16 UNUSED
// 17 UNUSED
// 18 UNUSED
#define PCI2C1  19
// 20 UNUSED
#define PCSSP0  21
#define PCTIM2  22
#define PCTIM3  23
#define PCUART2 24
#define PCUART3 25
#define PCI2C2  26
#define PCI2S   27
#define PCSDC   28
#define PCGPDMA 28
#define PCENET  30
#define PCUSB   31// PERIPHERAL POWER CONTROL BITS
// 0 UNUSED

#define PCTIM1   2
#define PCUART0  3
#define PCUART1  4
// 5 UNUSED
#define PCPWM1   6
#define PCI2C0   7
#define PCSPI    8
#define PCRTC    9
#define PCSSP1  10
#define PCEMC   11
#define PCAD    12
#define PCAN1   13
#define PCAN2   14
// 15 UNUSED
// 16 UNUSED
// 17 UNUSED
// 18 UNUSED
#define PCI2C1  19
// 20 UNUSED
#define PCSSP0  21
#define PCTIM2  22
#define PCTIM3  23
#define PCUART2 24
#define PCUART3 25
#define PCI2C2  26
#define PCI2S   27
#define PCSDC   28
#define PCGPDMA 28
#define PCENET  30

// PERIPH CLOCK BITS

#define PCLK_UART0  6
#define PCLK_UART1  8
#define PCLK_UART2 16
#define PCLK_UART3 18
#define PCLK_SSP1  20
#define PCLK_ADC   24

// **********************
// Interrupts
// **********************
#define EXT_INT 01

// pin mode bit locations
#define EXTINT0  20
#define EXTINT1  22
#define EXTINT2  24
#define EXTINT3  26

#define EXTINT0_CLEAR 0x00000001
#define EXTINT1_CLEAR 0x00000002
#define EXTINT2_CLEAR 0x00000004
#define EXTINT3_CLEAR 0x0000000

//==========================
// PORT 0 defs
//===========================

#define uP_AUX_TX       0     // UART3  
#define uP_AUX_RX       1     // UART3  
#define uP_DBG_TXD      2     // UART0
#define uP_DBG_RXD		3     // UART0
#define FRAM_CS         6     // SPI
#define SPI_SCK         7     // SPI
#define SPI_MISO        8     // SPI
#define SPI_MOSI        9     // SPI


#define XB_TX           15    // UART1
#define XB_RX           16    // UART1
#define XB_CTS          17    // UART1
#define ECHO_RX         17    // Ultasonic   (NOTE: TEMP Blue wire)
#define XB_RST          19    // Dout
#define XB_RTS          22    // UART1
#define uP_ADC_Vb       23    // ADC0.0
#define RSSI            24    // ADC0.1
#define uP_DEBUG        25    // P0.25


//==========================
// PORT 1 defs
//==========================

#define ECHO_IN         4     // TEMP  For bit bang only
#define TRIG_TX         8
#define LED_RED         15
#define LED_YELLOW      16
#define LED_GREEN       17
#define uP_MOD_x1       19
#define uP_MOD_x2       20
#define uP_MOD_x4       21
#define uP_MOD_x8       22

#define REV0            27
#define REV1            28
#define REV2            29

//==========================
// PORT 2 defs
//==========================
#define FRAM_WP            0
#define AUX_LED0           1   

#define MODEM_TX           8
#define MODEM_RX           9

//==========================
// PORT 3 defs
//==========================
#define START_ULTRA_SONIC  26 // Dout

//==========================
// PORT 4 defs
//==========================
#define DRY_OUT1        28    // Dout
#define DRY_OUT0        29    // Dout

//==========================
// Battery levels
//==========================
#define WARN_BATTERY_VOLTAGE     (12000)
#define LOW_BATTERY_VOLTAGE      (11600)
#define CRITICAL_BATTERY_VOLTAGE (11200)

//==========================
// Debug printf's for different systems
//==========================
//#define PACKET_PRINT		// Output debugs for packet Tx/Tx/Queues
//#define COMMAND_PRINT    // Output debugs in command.c file
#define SENSOR_PRINT 
//#define BEACON_PRINT
#define ASSET_PRINT
//#define RSSI_PRINT

#endif		// SHARED_DEFS_H

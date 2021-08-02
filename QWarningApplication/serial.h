#ifndef SERIAL_H
#define SERIAL_H
#include <LPC23xx.H>                   /* LPC23xx definitions               */
#include <stdio.h>
#include "shareddefs.h"

//#define DEBUG_USE_INTERRUPTS

#define FIFO_ENABLE 0x01

#define THRE_INTERRUPT_ENABLE 0x02
#define RX_DATA_AVAILABLE_INTERRUPT_ENABLE 0x01
#define RDR 0x01
#define THRE 0x20
#define TEMT 0x40


#define SUCCESS 	 1
#define ERROR 		-1

#define EIGHTBITS  	   0x03	// LCR select 8 bits
#define DATA_WIDTH      0	   // LCR bits 1:0 = data witdth
#define DLAB	   		7    	// LCR bit 7

/////
// BAUD = PCLK/(16*((256*DLM)+DLL)*(1+(divAddValue/MultValue))
// ignoring divAddValue and multValue
// BAUD = PCLK/(16*((256*DLM)+DLL))
// PCLK/BAUD = (16*((256*DLM)+DLL))
// PCLK/(BAUD*16) = (256*DLM)+DLL;
// DLM = (PCLK/(BAUD*16))/256)
// DLL = PCLK/(BAUD*16)-(256*DLM)
/////
#define DLL_9600_96MHZ		 			0x71	//MAGIC #: 9600bps@96000000 CCLK = PCLK	  
#define DLM_9600_96MHZ					2

#define DLL_115200_96MHZ	 			0x34	//MAGIC #: 115200bps@96000000 CCLK = PCLK	  
#define DLM_115200_96MHZ				0

#define DLL_9600_72MHZ		 			0xD4	//MAGIC #: 9600bps@72000000 CCLK = PCLK	  
#define DLM_9600_72MHZ					1	

#define DLL_19200_72MHZ		 			0xEA	//MAGIC #: 19200bps@72000000 CCLK = PCLK	  
#define DLM_19200_72MHZ					0	

#define DLL_115200_72MHZ	 			0x27	//MAGIC #: 115200bps@72000000 CCLK = PCLK	  
#define DLM_115200_72MHZ				0

#define DLL_115200_36MHZ  			   19	  //MAGIC #: 115200bps@36000000 CCLK = PCLK
#define DLM_115200_36MHZ				0    // Error = 0.0%

#define DLL_57600_36MHZ					39 	//MAGIC #: 57600bps@36000000 CCLK = PCLK
#define DLM_57600_36MHZ					0

#define DLL_19200_36MHZ					117	//MAGIC #: 19200bps@36000000 CCLK = PCLK
#define DLM_19200_36MHZ					0

#define DLL_9600_36MHZ					228	//MAGIC #: 19200bps@36000000 CCLK = PCLK
#define DLM_9600_36MHZ					0

#define DLL_1200_36MHZ					32 	//MAGIC #: 19200bps@36000000 CCLK = PCLK
#define DLM_1200_36MHZ					7


#define TXD0_SEL	4   		   // pinsel0
#define RXD0_SEL	6				// pinsel0
#define UART0_SEL	0x01		   // this value is shifted left by [T, R]xD0_SEL in pisel0 register

#define TXD1_SEL	30	 		   // **PINSEL0
#define RXD1_SEL	0	 			// **PINSEL1
#define UART1_SEL 0x01		   // ditto pinsel1

#define TXD2_SEL	20			   //pinsel0
#define RXD2_SEL	22			   //pinsel0
#define UART2_SEL	0x01		   // ...and so on

#define TXD3_SEL	24				// pinsel0
#define RXD3_SEL	26				// pinsel0
#define UART3_SEL	0x03		   // ...and so forth


// periph clock
#define PCLK_UART2 	16

// periph power
#define PCUART2 		24	

#define NO_PACKET_DATA 							-1

#define SEND_QUEUE_MAX_ENTRIES		30

#define SEND_QUEUE_IDLE_TIME				50

//********************
// **** PROTOTYPES

int sendchar      (int);

void serialInit(ePLATFORM_TYPE ePlatformType);
void assetSerialDoWork( void );
void radarSerialDoWork( void );
void serialDisableInterface(eINTERFACE eInterface);
void serialEnableInterface(eINTERFACE eInterface);
void serialRS485SendData(unsigned char nData);
unsigned char serialReadWireless(unsigned char* pData);
unsigned char serialWriteWireless(unsigned char nData);
//jgrvoid serialSendPacket(eINTERFACE eInterface, unsigned char* pPacket, int nPacketLength);
void serialWrite(eINTERFACE eInterface, unsigned char* pPacket, int nPacketLength);
void serialSendByteToAsset(unsigned char nData);
extern void serialSendByteToModem( unsigned char nData );

#endif		// SERIAL_H


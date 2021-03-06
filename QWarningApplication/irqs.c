/*****************************************************************************
 *   irq.c: Interrupt handler C file for NXP LPC23xx/24xx Family Microprocessors
 *
 *   Copyright(C) 2006, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2006.07.13  ver 1.00    Prelimnary version, first Release
 *
 ******************************************************************************/
#include "LPC23xx.h"                /* LPC23XX/24xx Peripheral Registers */
#include "shareddefs.h"
#include "sharedInterface.h"
#include <stdio.h>
#include "irqs.h"
#if FIQ
#include "timer.h"
#endif


/* Initialize the interrupt controller */
/******************************************************************************
** Function name:		init_VIC
**
** Descriptions:		Initialize VIC interrupt controller.
** parameters:			None
** Returned value:	None
**
******************************************************************************/
void init_VIC( void )
{
   uint32_t i = 0;
   uint32_t *vect_addr, *vect_prio;

   /* initialize VIC*/
   VICIntEnClr = 0xffffffff;
   VICVectAddr = 0;
   VICIntSelect = 0;

   /* set all the vector and vector control register to 0 */
   for( i = 0; i < VIC_SIZE; i++ )
   {
      vect_addr  = (uint32_t *)( VIC_BASE_ADDR + VECT_ADDR_INDEX + i * 4 );
      vect_prio  = (uint32_t *)( VIC_BASE_ADDR + VECT_PRIO_INDEX + i * 4 );
      *vect_addr = 0x0;
      *vect_prio = 0xF;
   }
}

/******************************************************************************
** Function name:		install_irq
**
** Descriptions:		Install interrupt handler
** parameters:			Interrupt number, interrupt handler address,
**						interrupt priority
** Returned value:		true or false, return false if IntNum is out of range
**
******************************************************************************/
BOOL install_irq( uint8_t IntNumber, void ( *HandlerAddr )(void) __irq, uint32_t Priority )
{
   uint32_t *vect_addr;
   uint32_t *vect_prio;

   VICIntEnClr = 1 << IntNumber;    /* Disable Interrupt */
   if( IntNumber >= VIC_SIZE )
   {
      return ( FALSE );
   }
   else
   {
      /* find first un-assigned VIC address for the handler */
      vect_addr    = (uint32_t *)( VIC_BASE_ADDR + VECT_ADDR_INDEX + IntNumber * 4 );
      vect_prio    = (uint32_t *)( VIC_BASE_ADDR + VECT_PRIO_INDEX + IntNumber * 4 );
      *vect_addr   = (uint32_t)HandlerAddr;           /* set interrupt vector */
      *vect_prio   = Priority;
      VICIntEnable = 1 << IntNumber;            /* Enable Interrupt */

      //printf("VICIntEnable[%lu]\n",VICIntEnable);
      return( TRUE );
   }
}


/******************************************************************************
**                            End Of File
******************************************************************************/


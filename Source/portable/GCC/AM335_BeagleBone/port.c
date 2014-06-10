/*
    FreeRTOS V7.0.1 - Copyright (C) 2011 Real Time Engineers Ltd.

    ***************************************************************************
    *                                                                         *
    * If you are:                                                             *
    *                                                                         *
    *    + New to FreeRTOS,                                                   *
    *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
    *    + Looking for basic training,                                        *
    *    + Wanting to improve your FreeRTOS skills and productivity           *
    *                                                                         *
    * then take a look at the FreeRTOS books - available as PDF or paperback  *
    *                                                                         *
    *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
    *                  http://www.FreeRTOS.org/Documentation                  *
    *                                                                         *
    * A pdf reference manual is also available.  Both are usually delivered   *
    * to your inbox within 20 minutes to two hours when purchased between 8am *
    * and 8pm GMT (although please allow up to 24 hours in case of            *
    * exceptional circumstances).  Thank you for your support!                *
    *                                                                         *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public 
    License and the FreeRTOS license exception along with FreeRTOS; if not it 
    can be viewed here: http://www.freertos.org/a00114.html and also obtained 
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/


/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM7 port.
 *
 * Components that can be compiled to either ARM or THUMB mode are
 * contained in this file.  The ISR routines, which can only be compiled
 * to ARM mode are contained in portISR.c.
 *----------------------------------------------------------*/


/* Standard includes. */
#include <stdlib.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "am335.h"

/* Constants required to setup the task context. */
#define portINITIAL_SPSR				( ( portSTACK_TYPE ) 0x1f ) /* System mode, ARM mode, interrupts enabled. */
#define portTHUMB_MODE_BIT				( ( portSTACK_TYPE ) 0x20 )
#define portINSTRUCTION_SIZE			( ( portSTACK_TYPE ) 4 )
#define portNO_CRITICAL_SECTION_NESTING	( ( portSTACK_TYPE ) 0 )

/*-----------------------------------------------------------*/
/* Helper functions */
extern void RegWrite( unsigned int base, unsigned int offset, unsigned int value);
extern void dumpinterrupts( void );
extern void dumptimer( void );
extern unsigned int RegRead( unsigned int base, unsigned int offset);


/*-----------------------------------------------------------*/

/* Setup the timer to generate the tick interrupts. */
static void prvSetupTimerInterrupt( void );

/* U-boot code uses Timer 1 */


/*--------------------------------------------------*/
/* 
 * The scheduler can only be started from ARM mode, so 
 * vPortISRStartFirstSTask() is defined in portISR.c. 
 */
extern void vPortISRStartFirstTask( void );

/*-----------------------------------------------------------*/

void U_DEF(void);
void P_FET(void);
void D_ABO(void);
void D_NOT(void);


/* 
 * Initialise the stack of a task to look exactly as if a call to 
 * portSAVE_CONTEXT had been called.
 *
 * See header file for description. 
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
portSTACK_TYPE *pxOriginalTOS;

	pxOriginalTOS = pxTopOfStack;

	/* Setup the initial stack of the task.  The stack is set exactly as 
	expected by the portRESTORE_CONTEXT() macro. */

	/* First on the stack is the return address - which in this case is the
	start of the task.  The offset is added to make the return address appear
	as it would within an IRQ ISR. */
	*pxTopOfStack = ( portSTACK_TYPE ) pxCode + portINSTRUCTION_SIZE;		
	pxTopOfStack--;

	*pxTopOfStack = ( portSTACK_TYPE ) 0xaaaaaaaa;	/* R14 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) pxOriginalTOS; /* Stack used when task starts goes in R13. */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x12121212;	/* R12 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x11111111;	/* R11 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x10101010;	/* R10 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x09090909;	/* R9 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x08080808;	/* R8 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x07070707;	/* R7 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x06060606;	/* R6 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x05050505;	/* R5 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x04040404;	/* R4 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x03030303;	/* R3 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x02020202;	/* R2 */
	pxTopOfStack--;	
	*pxTopOfStack = ( portSTACK_TYPE ) 0x01010101;	/* R1 */
	pxTopOfStack--;	

	/* When the task starts is will expect to find the function parameter in
	R0. */
	*pxTopOfStack = ( portSTACK_TYPE ) pvParameters; /* R0 */
	pxTopOfStack--;

	/* The last thing onto the stack is the status register, which is set for
	system mode, with interrupts enabled. */
	*pxTopOfStack = ( portSTACK_TYPE ) portINITIAL_SPSR;

	if( ( ( unsigned long ) pxCode & 0x01UL ) != 0x00 )
	{
		/* We want the task to start in thumb mode. */
		*pxTopOfStack |= portTHUMB_MODE_BIT;
	}

	pxTopOfStack--;

	/* Some optimisation levels use the stack differently to others.  This 
	means the interrupt flags cannot always be stored on the stack and will
	instead be stored in a variable, which is then saved as part of the
	tasks context. */
	*pxTopOfStack = portNO_CRITICAL_SECTION_NESTING;

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

portBASE_TYPE xPortStartScheduler( void )
{
	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
	prvSetupTimerInterrupt();
	/* Start the first task. */
	vPortISRStartFirstTask();	

	/* Should not get here! */
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the ARM port will require this function as there
	is nothing to return to.  */
}
/*-----------------------------------------------------------*/
void U_DEF(void)
{
    //printf("undefine \n");
}
void P_FET(void)
{
    //printf("prefetch\n");
}
void D_ABO(void)
{
    //printf("data abort\n");
}

void D_NOT(void)
{
    //printf("data abort\n");
}
/*
 * Setup the timer 0 to generate the tick interrupts at the required frequency.
 */
static void prvSetupTimerInterrupt( void )
{
	volatile int i;
    	volatile unsigned long ul; 
	unsigned long ulCompareMatch;
	extern void ( vIRQHandler )( void );
	extern void ( vPortYieldProcessor ) ( void );
	

	//portDISABLE_INTERRUPTS();

	/* Reset Interrupt Controller */
    /* Performing a software reset to trigger a Memory Protection Unit 
     * Interrupt Controller module reset */
	(*(REG32(MPU_INTC + INTCPS_SYSCONFIG))) = 0x00000002;
	while(((*(REG32(MPU_INTC + 0x14)))&1)!=1)	{} //wait for module to be OK
	for ( i=0; i<2000; i++){}
    /* Functional clock auto-idle mode : FuncFree */
	(*(REG32(MPU_INTC + INTCPS_IDLE))) = 0x00000001;
	for ( i=0; i<100; i++){}
	(*(REG32(MPU_INTC + 0x68))) = 0xFF;
	for ( i=0; i<100; i++){}
	(*(REG32(MPU_INTC + INTCPS_ILSR68))) = 0x0C;
	(*(REG32(MPU_INTC + INTCPS_ILSR96))) = 0x0C;
	(*(REG32(MPU_INTC + INTCPS_ILSR97))) = 0x0C;
	for ( i=0; i<100; i++){}




	/* Setup Interrupts */
	E_UNDEFINED = (long)U_DEF;
	E_PREFETCH = (long)P_FET;
	E_DATA_ABRT = (long)D_ABO;
	

	/* Enable Interrupt 68 and 96 and 97 which is used for DMTIMER 2 and GPIO0INTS*/
	(*(REG32(MPU_INTC + INTCPS_MIR_CLEAR2))) = ~(*(REG32(MPU_INTC + INTCPS_MIR2)))|0x10;
	(*(REG32(MPU_INTC + INTCPS_MIR_CLEAR3))) = ~(*(REG32(MPU_INTC + INTCPS_MIR3)))|0x1;
	(*(REG32(MPU_INTC + INTCPS_MIR_CLEAR3))) = ~(*(REG32(MPU_INTC + INTCPS_MIR3)))|0x2;
	for ( i=0; i<100; i++){}


	/* Calculate the match value required for our wanted tick rate */
	ulCompareMatch = configDMTIMER2_CLOCK_HZ / configTICK_RATE_HZ;

	//READ CM_PER offset 0, bit 14 to see if timer2 is active
	/* Set and enable DMTIMER2 */
	(*(REG32(CM_DPLL_REG + DMTIMER2_CLKSEL))) =0x1;
	 (*(REG32(CM_PER + CM_PER_TIMER2))) =0x2;
	for ( i=0; i<2000; i++){i++;}

	
	//(*(REG32(GPTI1 + GPTI_TIOCP_CFG))) = 0x2; // reset interface
	(*(REG32(DMTIMER2 + 0x10))) = 0x1; // reset interface

	for ( i=0; i<100; i++){}
	//(*(REG32(GPTI1 + GPTI_TCRR))) = 0; // initialize counter
	(*(REG32(DMTIMER2 + 0x3C))) = 0; // initialize counter
	for ( i=0; i<100; i++){}
	//(*(REG32(GPTI1 + GPTI_TMAR))) = ulCompareMatch; // load match value
	(*(REG32(DMTIMER2 + 0x4C))) = ulCompareMatch;
	for ( i=0; i<100; i++){}
	/* Clear pending matching interrupt (if any) */
	//(*(REG32(GPTI1 + GPTI_TISR))) = 0x1;
	(*(REG32(DMTIMER2 + 0x28))) = 0x1;
	for ( i=0; i<100; i++){}
	/* Enable matching interrupts */
	//(*(REG32(GPTI1 + GPTI_TIER))) = 0x1;
	(*(REG32(DMTIMER2 + 0x2C))) = 0x1;

	(*(REG32(DMTIMER2 + 0x40))) = 0x0; //reload value
	for ( i=0; i<100; i++){}
	/* Timer Control Register
	 * bit 0 -> start
	 * bit 1 -> autoreload
	 * bit 6 -> compare enabled
	 */
	(*(REG32(DMTIMER2 + 0x38))) = 0x43;
	for ( i=0; i<100; i++){}
	/* Reset the timer */
	//(*(REG32(GPTI1 + GPTI_TTGR))) = 0xFF;

	(*(REG32(DMTIMER2 + 0x44))) = 0xFF;
	for ( i=0; i<100; i++){}

	
}


/*-----------------------------------------------------------*/


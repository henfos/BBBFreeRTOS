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

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/

/* Standard includes. */
#include <stdlib.h>
#include <string.h>

#include "serial.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "am335.h"

int checkInt = 0;
short channel1_flag = FALSE;
short channel2_flag = FALSE;
short channel3_flag = FALSE;

static void prvSetupHardware( void );

void DATA_ABORT ( void ) __attribute__((naked));

/*-----------------------------------------------------------*/

static void vRespTask1(void *pvParameters)
{

	uint32_t xLastWakeTime;
	const uint32_t xFrequency = 1;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	int i=0;
	serial_puts(UART0_BASE,"task1\n");
	(*(REG32(GPIO0_BASE + GPIO_SETDATAOUT))) = PIN5|PIN4|PIN3;
	while(1)
	{
		if (channel1_flag == TRUE)	{
			channel1_flag = FALSE;
			i=0xAF;
			while(i--){;}
			(*(REG32(GPIO0_BASE + GPIO_CLEARDATAOUT))) = PIN5;
			i=0xAF;
			while(i--){;}
			(*(REG32(GPIO0_BASE + GPIO_SETDATAOUT))) = PIN5;
			//serial_puts(UART0_BASE,"r1\n");
		}
		if (channel2_flag == TRUE)	{
			channel2_flag = FALSE;
			i=0xAF;
			while(i--){;}
			(*(REG32(GPIO0_BASE + GPIO_CLEARDATAOUT))) = PIN4;
			i=0xAF;
			while(i--){;}
			(*(REG32(GPIO0_BASE + GPIO_SETDATAOUT))) = PIN4;
			//serial_puts(UART0_BASE,"r2\n");			
		}
		if (channel3_flag == TRUE)	{
			channel3_flag = FALSE;
			i=0xAF;
			while(i--){;}
			(*(REG32(GPIO0_BASE + GPIO_CLEARDATAOUT))) = PIN3;
			i=0xAF;
			while(i--){;}
			(*(REG32(GPIO0_BASE + GPIO_SETDATAOUT))) = PIN3;
			//serial_puts(UART0_BASE,"r3\r\n");
		}
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		//vTaskDelay(1);	
	}

}

/*static void vRespTask2(void *pvParameters)
{
	uint32_t xLastWakeTime;
	const uint32_t xFrequency = 1;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	const portTickType msDelay = 1;
	unsigned int c,ret,i;
	char buf[3];
	serial_puts(UART0_BASE,"task2\n");
	(*(REG32(GPIO0_BASE + GPIO_SETDATAOUT))) |= PIN4;
	while(1)
	{
		if (channel2_flag == TRUE)	{
			channel2_flag = FALSE;
			i=0x1FF;
			while(i--){;}
			(*(REG32(GPIO0_BASE + GPIO_CLEARDATAOUT))) |= PIN4;
			i=0xAF;
			while(i--){;}
			(*(REG32(GPIO0_BASE + GPIO_SETDATAOUT))) |= PIN4;
			//serial_puts(UART0_BASE,"r2\n");			
		}
		(*(REG32(GPIO0_BASE + GPIO_SETDATAOUT))) |= PIN4;
		//vTaskDelayUntil(&xLastWakeTime, xFrequency);
		vTaskDelay(1);
	}
}

static void vRespTask3(void *pvParameters)
{
	uint32_t xLastWakeTime;
	const uint32_t xFrequency = 1;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	const portTickType msDelay = 1;
	unsigned int i;
	serial_puts(UART0_BASE,"task3\r\n");
	(*(REG32(GPIO0_BASE + GPIO_SETDATAOUT))) |= PIN3;
	while(1)
	{
		if (channel3_flag == TRUE)	{
			channel3_flag = FALSE;
			i=0x1FF;
			while(i--){;}
			(*(REG32(GPIO0_BASE + GPIO_CLEARDATAOUT))) |= PIN3;
			i=0xAF;
			while(i--){;}
			(*(REG32(GPIO0_BASE + GPIO_SETDATAOUT))) |= PIN3;
			//serial_puts(UART0_BASE,"r3\r\n");
		}
		(*(REG32(GPIO0_BASE + GPIO_SETDATAOUT))) |= PIN3;
		//vTaskDelayUntil(&xLastWakeTime, xFrequency);
		vTaskDelay(1);
	}
}*/

static void vBlink(void *pvParameters)
{
	const portTickType msDelay = 500;
	unsigned int i;
	serial_puts(UART0_BASE,"blinktask\n");
	while(1)
	{

			(*(REG32(GPIO1_BASE + GPIO_CLEARDATAOUT))) |= PIN21;
			i=0x1FFF;
			while(i--){;}
			(*(REG32(GPIO1_BASE + GPIO_SETDATAOUT))) |= PIN21;
			i=0x1FFF;
			while(i--){;}
	}
}


void DATA_ABORT()	{
	serial_puts(UART0_BASE,"omg dataabort...\n");
	while(1){}
}

/*
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{   

	/* Initialise the LED outputs */
	prvSetupHardware();

	//INIT SERIAL
	init_serial(UART0_BASE);
	//init_serial(UART4_BASE);
	//INIT SERIAL END
	serial_puts(UART0_BASE,"Starting FreeRTOS\n");
	/*unsigned int test = (*(REG32(CONTROL_MODULE + 0x95c)));
	i = sprintf(buf,"%x\n",test);
	buf[i]='\0';
	serial_puts(buf);*/
	/*while(1)	{
		while ((*(REG32(UART0_BASE + 0x20)))==0){;}
			c = (*(REG32(UART0_BASE + 0x00)));
		serial_putc(UART0_BASE,c);	
	}*/


	xTaskCreate(vRespTask1,  ( signed char * ) "resp1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, ( xTaskHandle * ) NULL);

	/*xTaskCreate(vRespTask2,  ( signed char * ) "resp2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, ( xTaskHandle * ) NULL);

	xTaskCreate(vRespTask3,  ( signed char * ) "resp3", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, ( xTaskHandle * ) NULL);*/

	xTaskCreate(vBlink,  ( signed char * ) "BLINK1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, ( xTaskHandle * ) NULL);
	xTaskCreate(vBlink,  ( signed char * ) "BLINK2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, ( xTaskHandle * ) NULL);
	xTaskCreate(vBlink,  ( signed char * ) "BLINK3", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, ( xTaskHandle * ) NULL);

	vTaskStartScheduler();

	return 0;
}

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{

	/* Initialize GPIOs */

    /* BONE */
    /* Enabling the GPIO0 and GPIO1 clocks */
    (*(REG32(PRCM_REG + CM_PER_GPIO1_CLKCTRL))) =0x2;
    (*(REG32(CM_WKUP + 0x8))) =0x2;
    /* Enabling the UART0 and UART4 clocks */
    (*(REG32(CM_WKUP + 0xB4))) =0x2;
    (*(REG32(PRCM_REG + 0x78))) =0x2;
	
	//Setup GPIO pins
	(*(REG32(CONTROL_MODULE + 0x95c))) = 0x17;
	(*(REG32(CONTROL_MODULE + 0x958))) = 0x17;
	(*(REG32(CONTROL_MODULE + 0x954))) = 0x17;
	(*(REG32(CONTROL_MODULE + 0x950))) = 0x27;
	(*(REG32(CONTROL_MODULE + 0x984))) = 0x27;
	(*(REG32(CONTROL_MODULE + 0x980))) = 0x27;
	//Setup UART4 pins
	(*(REG32(CONTROL_MODULE + 0x870))) = 0x26;
	(*(REG32(CONTROL_MODULE + 0x874))) = 0x06;

	

    /* Controlling the output capability */
    (*(REG32(GPIO1_BASE+GPIO_OE))) = ~(PIN21|PIN22|PIN23|PIN24);  


	(*(REG32(GPIO0_BASE+0x30))) |= (PIN2 | PIN14 | PIN15);  //IRQ Status  
	(*(REG32(GPIO0_BASE+0x34))) |= (PIN2 | PIN14 | PIN15);  //IRQ Status  set
	(*(REG32(GPIO0_BASE+0x14C))) |= (PIN2 | PIN14 | PIN15);  //Falling edge
	(*(REG32(GPIO0_BASE+GPIO_OE))) = ~(PIN5 | PIN4 | PIN3);  
    (*(REG32(GPIO0_BASE + GPIO_SETDATAOUT))) |= (PIN5 | PIN4 | PIN3);
    //(*(REG32(GPIO0_BASE + GPIO_SETDATAOUT))) |= PIN2;
    /* Controlling the output capability */
    //(*(REG32(GPIO0_BASE+GPIO_OE))) = ~(PIN3|PIN4|PIN5);  

    /* Switch off the leds */ 
    (*(REG32(GPIO1_BASE+GPIO_CLEARDATAOUT))) = PIN24|PIN23|PIN22|PIN21; 
} 


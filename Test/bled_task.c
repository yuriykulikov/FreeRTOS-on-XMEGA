/* Compiler definitions include file. */
#include "avr_compiler.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* File headers. */
//#include "led.h"
//#include "usart_driver_RTOS.h"

//#include "exceptions.h"

/* Task header file */
//#include "usart_task.h"
//#include <string.h>

#include "port_driver.h"
//static void mdelay(uint16_t );

static void mdelay(uint16_t ms)
{
	uint32_t count;

	// Approximate the number of loop iterations needed.
	//count = sysclk_get_cpu_hz() / 6;
	count = 10000000 / 6;
	count *= ms;
	count /= 1000;

	do {
		asm("");
	} while (--count);
}

/*! \brief Example Blinking LED task
 *
 */

void bledTask( void *pvParameters )
{

	/* Task loops forever*/
	for (;;)
	{
		PORT_ClearPins( &PORTE, 0x0f); // ! clear pin = light with the Xplained board
		mdelay(500);
		PORT_SetPins( &PORTE, 0x0f);
		mdelay(500);
	}
}

void startbledTask (char cPriority)
{
	xTaskCreate(bledTask, (signed char*)"BLED", 100, NULL, cPriority, NULL);
}


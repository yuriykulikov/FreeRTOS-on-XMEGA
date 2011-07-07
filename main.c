/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA firmware for prototype SPI board source
 *
 *      This file contains example application
 *-----------------------------------------------------------------------------
 *      Naming conventions
 *      Code provided by Atmel is written in C convention, like this:
 *      CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );
 *
 *      Code provided by Real Time Engineers (FreeRTOS.org) is
 *      provided in Hungarian convention using CamelCase:
 *      vTaskStartScheduler();
 *
 *      My code, since I  am more a Java guy, is in CamelCase.
 *      Although, giving that there are no classes, I cannot name
 *      functions starting with action, like we do in Java - thing.doSomeStuff()
 *      Instead, first comes comes the thing to which action is related:
 *      ledGroupEventQueuePut(ledRGBEventQueue,BLUE,700);
 *-------------------------------------------------------------------------------
 *     
 * \author
 *      Universit�t Erlangen-Nurnberg
 *		LS Informationstechnik (Kommunikationselektronik)
 *		Yuriy Kulikov
 *      Support email: Yuriy.Kulikov.87@googlemail.com
 *****************************************************************************/

/* Compiler definitions include file. */
#include "avr_compiler.h"
#include <avr/sleep.h>
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
/* Atmel drivers */
#include "clksys_driver.h"
#include "pmic_driver.h"
/* File headers. */

//#include "led.h"
//#include "ledGroup.h"
//#include "usart_task.h"
#include "bled_task.h"
// This is global, because used in hooks
//LedGroup * ledRGB;
/* BADISR_vect is called when interrupt has occurred, but there is no ISR handler for it defined */
/*ISR (BADISR_vect){
	//stop execution and report error
	while(true) ledGroupSet(ledRGB, ORANGE);
}*/
int main( void )
{
	/*  Enable internal 32 MHz ring oscillator and wait until it's
	 *  stable. Set the 32 MHz ring oscillator as the main clock source.
	 */
	CLKSYS_Enable( OSC_RC32MEN_bm );
	CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc );
	do {} while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );

	// Setup output for the LEDs
	PORT_SetDirection( &PORTE, 0xff );
	PORT_SetPins( &PORTE, 0xff ); // /!\ set pin => switch *off* light on the Atmel A1-Xplained board
//	while(1)
//	{
//		PORT_ClearPins( &PORTE, 0x01);
//		mdelay(100);
//		PORT_SetPins( &PORTE, 0x01);
//		mdelay(100);
//	}


	/* Do all configuration and create all tasks and queues before scheduler is started.
	 * It is possible to put initialization of peripherals like displays into task functions
	 * (which will be executed after scheduler has started) if fast startup is needed.
	 * Interrupts are not enabled until the call of vTaskStartScheduler();
	 */

	//---------Use USART on PORTC----------------------------
//	UsartBuffer * usartFTDI = usartBufferInitialize(&USARTC0, BAUD9600, 128);
	// Report itself
//	usartBufferPutString(usartFTDI, "XMEGA ready",10);
	//---------Start LED task for testing purposes-----------
//	ledRGB = ledGroupInitialize(3);
//	ledGroupAdd(ledRGB, &PORTA, 0x20,1 );//R
//	ledGroupAdd(ledRGB, &PORTA, 0x10,1 );//G
//	ledGroupAdd(ledRGB, &PORTA, 0x08,1 );//B

//	LedGroupEventQueue * ledRGBEventQueue = startLedQueueProcessorTask(ledRGB,configLOW_PRIORITY, NULL);
//	ledGroupEventQueuePut(ledRGBEventQueue,BLUE,700);
//	ledGroupEventQueuePut(ledRGBEventQueue,SKY,700);
//	ledGroupEventQueuePut(ledRGBEventQueue,WHITE,700);
//	startBlinkingLedTask(ledRGBEventQueue,configLOW_PRIORITY, NULL);
	startbledTask(configNORMAL_PRIORITY);


	// Start USART task
//	startUsartTask(usartFTDI, ledRGBEventQueue, 128, configNORMAL_PRIORITY, NULL);

	// Enable PMIC interrupt level low
	PMIC_EnableLowLevel();
	/* Start scheduler. Creates idle task and returns if failed to create it.
	 * vTaskStartScheduler never returns during normal operation. If it has returned, probably there is
	 * not enough space in heap to allocate memory for the idle task, which means that all heap space is
	 * occupied by previous tasks and queues. Try to increase heap size configTOTAL_HEAP_SIZE in FreeRTOSConfig.h
	 * XMEGA port uses heap_1.c which doesn't support memory free. It is unlikely that XMEGA port will need to
	 * dynamically create tasks or queues. To ensure stable work, create ALL tasks and ALL queues before
	 * vTaskStartScheduler call. In this case we can be sure that heap size is enough.
	 * Interrupts would be enabled by calling sei();*/
	vTaskStartScheduler();

	/* stop execution and report error */
	while(true) ledGroupSet(ledRGB, PINK);
	return 0;
}
/* Prototype */
void vApplicationIdleHook( void );
/* This function is only called when there are no tasks to execute, except for the idle task. Most commonly it
 * is used to schedule co-routines or do some unimportant jobs. It is also great to put sleep(); to save
 * power. Microcontroller will stop code execution until the next interrupt from tick timer or peripheral.
 * configUSE_IDLE_HOOK should be defined as 1 to use IdleHook
 */
void vApplicationIdleHook( void )
{
   /* Go to sleep mode if there are no active tasks	*/
	sleep_mode();
}
/* Prototype */
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName );
	/* This function is called immediately after task context is saved into stack. This is the case
	 * when stack contains biggest amount of data. Hook function checks if there is a stack overflow
	 * for the current (switched) task. Parameters could be used for debug output.
	 * configCHECK_FOR_STACK_OVERFLOW should be defined as 1 to use StackOverflowHook.
	 */
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName )
{
	/* stop execution and report error */
	while(true) ledGroupSet(ledRGB, RED);
}
void vApplicationTickHook( void );
/* This function is called during the tick interrupt. configUSE_TICK_HOOK should be defined as 1.*/
void vApplicationTickHook( void )
{
/* Tick hook could be used to implement timer functionality*/
}

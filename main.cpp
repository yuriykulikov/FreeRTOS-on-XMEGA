/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA firmware for prototype board source
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
 *      Universität Erlangen-Nurnberg
 *		LS Informationstechnik (Kommunikationselektronik)
 *		Yuriy Kulikov
 *      Support email: Yuriy.Kulikov.87@googlemail.com
 *****************************************************************************/
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

extern "C" {
#include "avr_compiler.h"
#include <avr/sleep.h>
/* drivers */
#include "clksys_driver.h"
#include "pmic_driver.h"
#include "wdt_driver.h"
/* File headers. */
#include "strings.h"
}

#include "Looper.h"
#include "ExampleHandler.h"
#include "CommandInterpreter.h"
#include "CommandInterpreterThread.h"
#include "LedProcessorThread.h"
#include "Leds.h"
#include "Spi.h"
#include "Serial.h"
#include "SpiSlaveThread.h"

/** This is global, because used in hooks */
LedGroup ledRGB = LedGroup(3);
/** BADISR_vect is called when interrupt has occurred, but there is no ISR handler for it defined */
extern "C" ISR (BADISR_vect) {
	//stop execution and report error
	while(true) ledRGB.set(ORANGE);
}

/**
 * Callback function that will reset the WDT
 * @param xTimer
 */
extern "C" void watchdogTimerCallback( xTimerHandle xTimer ) {
	WDT_Reset();
}

/**
 * Main is used to:
 * 	Initialize driver structures
 * 	Create tasks
 * 	Pass driver structures to tasks
 */
int main( void ) {
    /*
     * Enable internal 32 MHz ring oscillator and wait until it's
     * stable. Set the 32 MHz ring oscillator as the main clock source.
     */
    CLKSYS_Enable( OSC_RC32MEN_bm );
    CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc );
    do {} while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );
    CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );

    //Enable watchdog timer, which will be reset by timer
    WDT_EnableAndSetTimeout( WDT_PER_1KCLK_gc );
    /*
     * Do all configuration and create all tasks and queues before scheduler is started.
     * It is possible to put initialization of peripherals like displays into task functions
     * (which will be executed after scheduler has started) if fast startup is needed.
     * Interrupts are not enabled until the call of vTaskStartScheduler();
     */
    // Enable the Round-Robin Scheduling scheme.Round-Robin scheme ensures that no low-level
    // interrupts are “starved”, as the priority changes continuously
    PMIC_EnableRoundRobin();

    //Create and start the timer, which will reset Watch Dog Timer
    xTimerStart(xTimerCreate((signed char*)"WDT",500, pdTRUE, 0, watchdogTimerCallback), 0);
    //---------Use USART on PORTC----------------------------
    Serial usartFTDI = Serial(&USARTE0, BAUD9600, 128, 10);

    // Initialize SPI slave on port D
    SpiSlave spiSlave = SpiSlave(&SPIC,false,SPI_MODE_1_gc,64);
    // Initialize SPI master on port C
    SpiMaster spiMaster = SpiMaster(&SPID, false, SPI_MODE_1_gc, false, SPI_PRESCALER_DIV4_gc);
    SpiDevice spiDevice = SpiDevice(&spiMaster, &PORTD, SPI_SS_bm);

    //---------Start LED task for testing purposes-----------
    ledRGB = LedGroup(3);
    ledRGB.add(&PORTF, 0x04,1 );//R
    ledRGB.add(&PORTF, 0x08,1 );//G
    ledRGB.add(&PORTF, 0x02,1 );//B
    ledRGB.set(BLUE);
    LedProcessorThread ledRGBEThread = LedProcessorThread(&ledRGB, GREEN, 500, "RGB", 64, configLOW_PRIORITY);

    LedGroup ledString = LedGroup(7);
    ledString.add(&PORTA, 0x02, 0);
    ledString.add(&PORTA, 0x04, 0);
    ledString.add(&PORTA, 0x08, 0);
    ledString.add(&PORTA, 0x10, 0);
    ledString.add(&PORTA, 0x20, 0);
    ledString.add(&PORTA, 0x40, 0);
    ledString.add(&PORTA, 0x80, 0);
    LedProcessorThread ledStringThread = LedProcessorThread(&ledString, "STR", 64, configLOW_PRIORITY);

    // ***** Start main Looper
    Looper looper = Looper(10, "LPR", 750, configNORMAL_PRIORITY);
    //XXX why it is not working if on the heap not on the stack?
    //ExampleHandler *exampleHandler = (ExampleHandler*) pvPortMalloc(sizeof(ExampleHandler));
    //*exampleHandler = ExampleHandler(looper, spiDevice, ledStringQueue, usartFTDI);
    ExampleHandler exampleHandler = ExampleHandler(&looper, &spiDevice, &ledStringThread, &usartFTDI);
    // ****** Register commands for the interpreter
    CommandInterpreter interpreter = CommandInterpreter();
    interpreter.registerCommand(Strings_SpiExampleCmd, Strings_SpiExampleCmdDesc, &exampleHandler, EVENT_RUN_SPI_TEST);
    interpreter.registerCommand(Strings_BlinkCmd, Strings_BlinkCmdDesc, &exampleHandler, EVENT_BLINK);
    CommandInterpreterThread cmdIntThreadFTDI = CommandInterpreterThread(&interpreter, 32, &usartFTDI, "I12", 128, configNORMAL_PRIORITY);

    // ****** Start stand-alone tasks
    SpiSlaveThread spiSlaveThread = SpiSlaveThread(&spiSlave, &usartFTDI, "SLV", 128, configLOW_PRIORITY);

    /* Start scheduler. Creates idle task and returns if failed to create it.
     * vTaskStartScheduler never returns during normal operation. It is unlikely that XMEGA port will need to
     * dynamically create tasks or queues. To ensure stable work, create ALL tasks and ALL queues before
     * vTaskStartScheduler call.
     * Interrupts would be enabled by calling PMIC_EnableLowLevel();*/
    vTaskStartScheduler();

    /* Should never get here, stop execution and report error */
    while(true) ledRGB.set(PINK);
    return 0;
}

/**
 * This function is only called when there are no tasks to execute, except for the idle task. Most commonly it
 * is used to schedule co-routines or do some unimportant jobs. It is also great to put sleep(); to save
 * power. Microcontroller will stop code execution until the next interrupt from tick timer or peripheral.
 * configUSE_IDLE_HOOK should be defined as 1 to use IdleHook
 */
extern "C" void vApplicationIdleHook( void ) {
    // Go to sleep mode if there are no active tasks
    sleep_mode();
}

/**
 * This function is called immediately after task context is saved into stack. This is the case
 * when stack contains biggest amount of data. Hook function checks if there is a stack overflow
 * for the current (switched) task. Parameters could be used for debug output.
 * configCHECK_FOR_STACK_OVERFLOW should be defined as 1 to use StackOverflowHook.
 */
extern "C" void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName ) {
    while(true) ledRGB.set(RED);
}

/**
 * There is not enough space in heap to allocate memory, which means that all heap space is
 * occupied by previous tasks and queues. Try to increase heap size configTOTAL_HEAP_SIZE in FreeRTOSConfig.h
 * XMEGA port uses heap_1.c which doesn't support memory free.
 * configUSE_MALLOC_FAILED_HOOK should be defined as 1 to use vApplicationMallocFailedHook()
 */
extern "C" void vApplicationMallocFailedHook() {
    while(true) ledRGB.set(PINK);
}
/**
 * This function will be called if a pure virtual method is invoked. It should never happen.
 */
extern "C" void __cxa_pure_virtual(void) {
    while(true) ledRGB.set(BLUE);
}

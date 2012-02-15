/* This file has been prepared for Doxygen automatic documentation generation.*/
/** \file *********************************************************************
 *
 * @brief
 *      XMEGA USART FreeRTOS driver source file.
 *
 *      This file contains the function implementations of the XMEGA interrupt
 *      FreeRTOS-based driver.
 *
 * \par Application note:
 *      AVR1307: Using the XMEGA USART
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *  \author
 *   	Universität Erlangen-Nürnberg
 *		LS Informationstechnik (Kommunikationselektronik)
 *		Yuriy Kulikov
 *      Support email: Yuriy.Kulikov.87@googlemail.com
 *
 * $Revision: 1694 $
 * $Date: 2008-07-29 14:21:58 +0200 (ti, 29 jul 2008) $  \n
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
extern "C" {
#include "port_driver.h"
#include "usart.h"
#include <avr/pgmspace.h>
}
#include "Serial.h"

//Structures, representing uart and its buffer. for internal use.
//Memory allocated dynamically
Serial * usartC;
Serial * usartD;
Serial * usartE;

ISR(USARTC0_RXC_vect){
    //TODO declare ISR as naked, add context switch
    uint8_t isHigherPriorityTaskWoken = usartC->USART_RXComplete();
}
ISR(USARTD0_RXC_vect){
    //TODO declare ISR as naked, add context switch
    uint8_t isHigherPriorityTaskWoken =  usartD->USART_RXComplete();
}
ISR(USARTE0_RXC_vect){
    //TODO declare ISR as naked, add context switch
    uint8_t isHigherPriorityTaskWoken =  usartE->USART_RXComplete();
}

ISR(USARTC0_DRE_vect){usartC->USART_DataRegEmpty();}
ISR(USARTD0_DRE_vect){usartD->USART_DataRegEmpty();}
ISR(USARTE0_DRE_vect){usartE->USART_DataRegEmpty();}

/**
 * @brief Initializes buffer and selects what USART module to use.
 *
 * This function is a "constructor", it allocates memory,
 * initializes receive and transmit buffer and selects what USART module to use,
 * and stores the data register empty interrupt level.
 * @param module hardware module to use
 * @param baudrate
 * @param bufferSize
 * @param ticksToWait - Default wait time
 * @return pointer to the Usart software module
 */
Serial::Serial(USART_t *module, Baudrate baudrate, uint8_t bufferSize, int ticksToWait) {
    PORT_t * port;
    //switch by pointer to usart structure, which will be used. Usually it is not a good idea to
    //switch by pointer, but in our case pointers are defined by hardware
    if (module == &USARTC0) {
        //copy pointer pUsartBuffer to global pUsartBufferC, which is use to handle interrupts
        usartC = this;
        //Since usart is on the port C, we will need to use PORTC
        port = &PORTC;
    } else if (module == &USARTD0) {
        //copy pointer pUsartBuffer to global pUsartBufferD, which is use to handle interrupts
        usartD = this;
        //Since usart is on the port D, we will need to use PORTD
        port = &PORTD;
    } else if (module == &USARTE0) {
        //copy pointer pUsartBuffer to global pUsartBufferD, which is use to handle interrupts
        usartE = this;
        //Since usart is on the port E, we will need to use PORTE
        port = &PORTE;
    }

    /* (TX0) as output. */
    port->DIRSET = PIN3_bm;
    /* (RX0) as input. */
    port->DIRCLR = PIN2_bm;
    //totempole and pullup
    PORT_ConfigurePins( port,PIN3_bm,false,false,PORT_OPC_PULLUP_gc,PORT_ISC_BOTHEDGES_gc );
    /* Initialize buffers. Create a queue (allocate memory) and store queue handle in Usart
     * On XMEGA port create all queues before vStartTaskScheduler() to ensure that heap size is enough */
    /* Store pointer to USART module */
    this->module = module;
    /*Store DRE level so we will know which level to enable when we put data and want it to be sent. */
    this->dreIntLevel = USART_DREINTLVL_LO_gc;
    // store default ticksToWait value - used in Dflt functions
    this->ticksToWait = ticksToWait;
    /* @brief  Receive buffer size: 2,4,8,16,32,64,128 bytes. */
    RXqueue = xQueueCreate(bufferSize,sizeof(char));
    TXqueue = xQueueCreate(bufferSize,sizeof(char));

    /* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
    USART_Format_Set(module, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
    /* Enable RXC interrupt. */
    USART_RxdInterruptLevel_Set(module, USART_RXCINTLVL_LO_gc);

    //http://prototalk.net/forums/showthread.php?t=188
    switch (baudrate) {
    case BAUD9600:
        USART_Baudrate_Set(module, 3317 , -4);
        break;
    default:
        //9600
        USART_Baudrate_Set(module, 3317 , -4);
        break;
    }

    /* Enable both RX and TX. */
    USART_Rx_Enable(module);
    USART_Tx_Enable(module);
}

/** @brief Put data (5-8 bit character).
 *
 *  Stores data byte in TX software buffer and enables DRE interrupt if there
 *  is free space in the TX software buffer.
 * @param usart
 * @param data The data to send
 * @param ticksToWait Amount of RTOS ticks (1 ms default) to wait if there is space in queue
 * @return pdTRUE is success, pdFALSE if queue was full and ticksToWait elapsed
 */
int8_t Serial::putByte(uint8_t data) {
    uint8_t tempCTRLA;
    signed char queueSendResult = xQueueSendToBack(TXqueue, &data, ticksToWait);
    /* If we successfully loaded byte to queue */
    if (queueSendResult == pdPASS) {
        /* Enable DRE interrupt. */
        tempCTRLA = module->CTRLA;
        tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | dreIntLevel;
        module->CTRLA = tempCTRLA;
        return pdPASS;
    } else {
        return pdFAIL;
    }
}

/** @brief Get received data
 *
 *  Returns pdTRUE is data is available and puts byte into &receivedChar variable
 *
 *  @param usart_struct       The USART_struct_t struct instance.
 *	@param receivedChar       Pointer to char variable for to save result.
 *	@param xTicksToWait       Amount of RTOS ticks (1 ms default) to wait if there is data in queue.
 *  @return					  Success.
 */
int8_t Serial::getByte(char * receivedChar, int ticks) {
    return xQueueReceive(RXqueue, receivedChar, ticks);
}
int8_t Serial::getByte(char * receivedChar) {
    return xQueueReceive(RXqueue, receivedChar, ticksToWait);
}
/** @brief Send string via Usart
 *
 *  Stores data string in TX software buffer and enables DRE interrupt if there
 *  is free space in the TX software buffer.
 *
 *  @param usart_struct The USART_struct_t struct instance.
 *  @param string       The string to send.
 *  @param xTicksToWait       Amount of RTOS ticks (1 ms default) to wait if there is space in queue.
 */
int8_t Serial::putString(const char *string)
{
    //send the whole string. Note that if buffer is full, USART_TXBuffer_PutByte will do nothing
    while (*string) {
        int8_t putByteResult = putByte(*string++);
        if (putByteResult == pdFAIL) return pdFAIL;
    }
    return pdPASS;
}
/** @brief Send program memory string via Usart
 *
 *  Stores data string in TX software buffer and enables DRE interrupt if there
 *  is free space in the TX software buffer.
 *  String is taken from the program memory.
 *
 *  @param usart_struct The USART_struct_t struct instance.
 *  @param string       The string to send.
 *  @param xTicksToWait       Amount of RTOS ticks (1 ms default) to wait if there is space in queue.
 */
int8_t Serial::putPgmString(const char *progmem_s)
{
    register char c;
    while ( (c = pgm_read_byte(progmem_s++)) ) {
        int8_t putByteResult =  putByte(c);
        if (putByteResult == pdFAIL) return pdFAIL;
    }
    return pdPASS;
}
/** @brief Put data (5-8 bit character).
 *
 *  Stores data integer represented as string in TX software buffer and enables DRE interrupt if there
 *  is free space in the TX software buffer.
 *
 *  @param usart Usart software abstraction structure
 *  @param Int       The integer to send.
 *  @param radix	Integer basis - 10 for decimal, 16 for hex
 *  @param xTicksToWait
 */
int8_t Serial::putInt(int16_t Int,int16_t radix) {
    char str[10];
    return putString(itoa(Int,str,radix));
}

/**
 * Receive complete interrupt service routine
 *
 * Stores received data in RX software buffer.
 * @param Usart software abstraction structure
 * @return xHigherPriorityTaskWoken
 */
signed char Serial::USART_RXComplete() {
    /* We have to check is we have woke higher priority task, because we post to
     * queue and high priority task might be blocked waiting for items appear on
     * this queue */
    signed char xHigherPriorityTaskWoken = pdFALSE;
    signed char cChar;
    /* Get the character and post it on the queue of Rxed characters.
     If the post causes a task to wake force a context switch as the woken task
     may have a higher priority than the task we have interrupted. */
    cChar = module->DATA;
    xQueueSendToBackFromISR( RXqueue, &cChar, &xHigherPriorityTaskWoken );
    return xHigherPriorityTaskWoken;
}

/** @brief Data Register Empty Interrupt Service Routine.
 *
 *  Data Register Empty Interrupt Service Routine. Transmits one byte from TX software buffer.
 *  Disables DRE interrupt if buffer is empty.
 *
 * @param Usart software abstraction structure
 * @return xHigherPriorityTaskWoken
 */
signed char Serial::USART_DataRegEmpty() {
    signed char cChar, cTaskWoken;
    if( xQueueReceiveFromISR( TXqueue, &cChar, &cTaskWoken ) == pdTRUE )
    {
        /* Send the next character queued for Tx. */
        module->DATA = cChar;
    } else {
        /* Queue empty, nothing to send. */
        /* Disable DRE interrupts. */
        uint8_t tempCTRLA = module->CTRLA;
        tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
        module->CTRLA = tempCTRLA;
    }
    return cTaskWoken;
}

/*
    FreeRTOS V7.0.2 - Copyright (C) 2011 Real Time Engineers Ltd.
	

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
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
#define DEBUG
//#undef DEBUG

extern "C" {
/* Standard includes. */
#include <string.h>
#include <avr/pgmspace.h>
#include "strings.h"
}
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Utils includes. */
#include "CommandInterpreter.h"
#include "CommandInterpreterThread.h"
#include "Thread.h"
#include "Serial.h"

CommandInterpreterThread::CommandInterpreterThread(CommandInterpreter *interpreter, uint8_t commandInputLen, Serial *serial, const char *name, unsigned short stackDepth, char priority)
:Thread(name, stackDepth, priority){
    commandInputBuffer = (char *) pvPortMalloc( sizeof(char)*commandInputLen);
    this->interpreter = interpreter;
    this->serial = serial;
}

void CommandInterpreterThread::run() {
    char receivedChar='#';
    /* Task loops forever*/
    for (;;) {
        //Empty the string first
        strcpy(commandInputBuffer,"");
        // Wait until the first symbol unblocks the task
        serial->getByte(&receivedChar,portMAX_DELAY);
        strncat(commandInputBuffer,&receivedChar,1);
        //Read string from queue, while data is available and put it into string
        // This loop will be over, when there is either ; or \n is received, or queue is empty for 200 ms
        while (serial->getByte(&receivedChar,200)) {
            if (receivedChar == ';') break;
            if (receivedChar == '\n') break;
            strncat(commandInputBuffer,&receivedChar,1);
        }

        interpreter->processCommand(commandInputBuffer, serial);
    }
}

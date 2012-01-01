/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA board-specific functions and tasks source.
 *
 *      This file contains functions to use led events. Led event is a bitmask to be lit,
 *      and time for which it should stay.
 *     
 * \author
 *      Universitaet Erlangen-Nuernberg
 *		LS Informationstechnik (Kommunikationselektronik)
 *		Yuriy Kulikov
 *      Support email: Yuriy.Kulikov.87@googlemail.com
 *
 * \par Documentation
 *     http://www.FreeRTOS.org - Documentation, latest information.
 *
 *****************************************************************************/

/* Compiler definitions include file. */
extern "C" {
#include "avr_compiler.h"
}
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/* File headers. */
#include "LedProcessorThread.h"
#include "Thread.h"
#include "Leds.h"
LedProcessorThread::LedProcessorThread(LedGroup * ledGroup, const char *name, unsigned short stackDepth, char priority)
:Thread(name, stackDepth, priority){
    this->ledGroup = ledGroup;
    this->queueHandle=xQueueCreate(8, sizeof(LedsEvent));
    this->idleBitmask = 0;
    this->idleDuration = portMAX_DELAY;
}

LedProcessorThread::LedProcessorThread(LedGroup * ledGroup, uint8_t bitmask, uint16_t duration, const char *name, unsigned short stackDepth, char priority)
:Thread(name, stackDepth, priority) {
    this->ledGroup = ledGroup;
    this->queueHandle=xQueueCreate(8, sizeof(LedsEvent));
    this->idleBitmask = bitmask;
    this->idleDuration = duration;
}

void LedProcessorThread::post(uint8_t bitmask, uint16_t duration) {
    //Create new empty LED structure
    //This is a local variable, it will exist only until the function return.
    LedsEvent newLedEvent;
    //Set value according to parameters
    newLedEvent.bitmask=bitmask;
    newLedEvent.duration=duration;
    /* Send  event data to queue. After this thread will be unblocked as soon as
     * all higher priority tasks get blocked, delayed, suspended - whatever.*/
    xQueueSendToBack(queueHandle,&newLedEvent,0);
}

void  LedProcessorThread::run() {
    LedsEvent newLedEvent;
    if (idleBitmask != 0) {
        ledGroup->set(SKY);
        vTaskDelay(500);
        ledGroup->set(NONE);
    }
    /* Cycle for ever*/
    while(true) {
        //Check while there are new events on the queue. Block if nothing.
        if (xQueueReceive(queueHandle, &newLedEvent, idleDuration ) == pdPASS ) {
            ledGroup->set(newLedEvent.bitmask);
            vTaskDelay(newLedEvent.duration);
            ledGroup->set(NONE);
        } else if (idleBitmask != 0) {
            // nothing was fetched from the queue and we have a default setting
            ledGroup->set(idleBitmask);
            vTaskDelay(idleDuration);
            ledGroup->set(NONE);
        }
    }
}

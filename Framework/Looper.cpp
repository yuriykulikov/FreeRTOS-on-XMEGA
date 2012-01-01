/*
 * Looper.c
 *
 *  Created on: Nov 20, 2011
 *      Author: Yuriy
 */
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Handler.h"
#include "Looper.h"
#include "ExampleHandler.h"
#include "Thread.h"

Looper::Looper(uint8_t messageQueueSize, const char *name, unsigned short stackDepth, char priority)
:Thread(name, stackDepth, priority) {
    messageQueue = xQueueCreate(messageQueueSize, sizeof(Message));
}

void Looper::run() {
    Message msg;
    //Infinite loop
    for (;;) {
        if (xQueueReceive(messageQueue, &msg, portMAX_DELAY)) {
            // Call handleMessage from the handler
            msg.handler->handleMessage(msg);
        }
    }
}

xQueueHandle Looper::getMessageQueue(){
    return messageQueue;
}

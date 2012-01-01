/*
 * Thread.cpp
 *
 *  Created on: Dec 29, 2011
 *      Author: Yuriy
 */
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

#include "Thread.h"

extern "C" void pvTaskCode(void *pvParameters) {
    (static_cast<Thread*>(pvParameters))->run();
}

Thread::Thread(const char *name, unsigned short stackDepth, char priority) {
    xTaskCreate(pvTaskCode, (const signed char *) name, stackDepth, (void*) this, priority, &taskHandle);
}

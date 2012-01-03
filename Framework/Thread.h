/*
 * Thread.h
 *
 *  Created on: Dec 29, 2011
 *      Author: Yuriy
 */

#ifndef THREAD_H_
#define THREAD_H_

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

class Thread {
private:
    xTaskHandle taskHandle;
public:
    Thread(const char *name, unsigned short stackDepth, char priority);
    virtual void run() =0;
};

#endif /* THREAD_H_ */

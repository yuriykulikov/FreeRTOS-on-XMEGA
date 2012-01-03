/*
 * Looper.h
 *
 *  Created on: Nov 20, 2011
 *      Author: Yuriy
 */

#ifndef LOOPER_H_
#define LOOPER_H_
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Thread.h"
class Looper : public Thread {
private:
	/** Queue on which handler posts messages */
	xQueueHandle messageQueue;
public:
	Looper(uint8_t messageQueueSize, const char *name, unsigned short stackDepth, char priority);
	xQueueHandle getMessageQueue();
	void run();
};

#endif /* LOOPER_H_ */

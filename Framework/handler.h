/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  Handler
 *
 *      This file contains an utility called handler. It is used to post
 *      messages to be handled by the task immediately or with a delay.
 *      A high-priority task, like command line interpreter should post
 *      messages to the handler.
 *
 * \author
 *      Universitat Erlangen-Nurnberg
 *		LS Informationstechnik (Kommunikationselektronik)
 *		Yuriy Kulikov
 *      Support email: Yuriy.Kulikov.87@googlemail.com
 *      Created on: Sep 27, 2011
 *
 *****************************************************************************/
#ifndef HANDLER_H_
#define HANDLER_H_

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Looper.h"

typedef struct MESSAGE Message;

class Handler {
private:
    /** Queue on which handler posts messages */
    xQueueHandle messageQueue;
public:
    Handler (Looper *looper);
    virtual void handleMessage(Message msg) =0;
    bool sendMessage(Message msg);
    bool sendMessage(char what);
    bool sendMessage(char what, char arg1, char arg2);
    bool sendMessage(char what, char arg1, char arg2, void *ptr);
    bool sendMessageFromISR(Message msg);
    bool sendMessageFromISR(char what);
    bool sendMessageFromISR(char what, char arg1, char arg2);
    bool sendMessageFromISR(char what, char arg1, char arg2, void *ptr);
};

struct MESSAGE {
	/** Handler responsible for handling this message */
	Handler *handler;
	/** What message is about */
	char what;
	/** First argument */
	char arg1;
	/** First argument */
	char arg2;
	/** Pointer to the allocated memory. Handler should cast to the proper type,
	 * according to the message.what */
	void *ptr;
};

#endif /* HANDLER_H_ */

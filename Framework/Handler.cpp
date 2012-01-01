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

/* Compiler definitions include file. */
extern "C" {
#include "avr_compiler.h"
#include <string.h>
}
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Task header file */
#include "Handler.h"
#include "Looper.h"

/**
 * Creates a handler, which has to be bind to the looper task
 * @param looper
 * @param handleMessage
 * @param context
 */
Handler::Handler(Looper *looper) {
    messageQueue = looper->getMessageQueue();
}

bool Handler::sendMessage(Message msg) {
    return xQueueSend(messageQueue, &msg, 0);
}

/**
 * Post message with arguments and a pointer to allocated data
 * @param handler
 * @param what
 * @param arg1
 * @param arg2
 * @param ptr
 */
bool Handler::sendMessage(char what, char arg1, char arg2, void *ptr) {
    Message msg;
    msg.handler = this;
    msg.what = what;
    msg.arg1 = arg1;
    msg.arg2 = arg2;
    msg.ptr = ptr;
    return xQueueSend(messageQueue, &msg, 0);
}

/**
 * Post empty message
 * @param handler
 * @param what
 */
bool Handler::sendMessage(char what) {
    return sendMessage(what, NULL, NULL, NULL);
}

/**
 * Post message with arguments
 * @param handler
 * @param what
 * @param arg1
 * @param arg2
 */
bool Handler::sendMessage(char what, char arg1, char arg2) {
    return sendMessage(what, arg1, arg2, NULL);
}



/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA board header
 *
 *      This file contains board defenitions and board-specific task and function prototypes. 
 *
 *
 * \par Documentation
 *     http://www.FreeRTOS.org - Documentation, latest information.
 *
 * \par Application note:
 *      AVR1000: Getting Started Writing C-code for XMEGA
 *
 * \author
 *      Universität Erlangen-Nürnberg
 *		LS Informationstechnik (Kommunikationselektronik)
 *		Yuriy Kulikov
 *      Support email: Yuriy.Kulikov.87@googlemail.com
 *
 *****************************************************************************/
#ifndef LED_H
#define LED_H

#include "avr_compiler.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Leds.h"
#include "Thread.h"

/* Led event, described by bitmask and duration */
typedef struct LEDS_EVENT {
    uint16_t duration;
    uint8_t bitmask;
} LedsEvent;

class LedProcessorThread: public Thread {
private:
    LedGroup * ledGroup;
    xQueueHandle queueHandle;
    uint8_t idleBitmask;
    uint16_t idleDuration;
public:
    LedProcessorThread(LedGroup * ledGroup, const char *name, unsigned short stackDepth, char priority);
    LedProcessorThread(LedGroup * ledGroup, uint8_t bitmask, uint16_t duration, const char *name, unsigned short stackDepth, char priority);
    void run();
    void post (uint8_t bitmask, uint16_t duration);
};
#endif

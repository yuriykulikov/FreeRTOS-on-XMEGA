/*
 * ExmapleHandler.h
 *
 *  Created on: Oct 26, 2011
 *      Author: Yuriy
 */

#ifndef EXAMPLE_HANDLER_H_
#define EXAMPLE_HANDLER_H_

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "Handler.h"
#include "LedProcessorThread.h"
#include "Serial.h"
#include "Spi.h"

#define EVENT_RUN_SPI_TEST 1
#define EVENT_BLINK 2

/** Used to store context of the handler */
class ExampleHandler: public Handler {
private:
    /** Pointer to SpiSlave to use */
    SpiDevice *master;
    LedProcessorThread *led;
    Serial *debugSerial;
public:
    ExampleHandler(Looper *looper, SpiDevice *spiMaster, LedProcessorThread *led, Serial *debugUsart);
    void handleMessage(Message msg);
};

#endif /* EXAMPLE_HANDLER_H_ */

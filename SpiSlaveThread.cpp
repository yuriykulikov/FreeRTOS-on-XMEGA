/* This file has been prepared for Doxygen automatic documentation generation.*/
/**@file *********************************************************************
 *
 * @brief  XMEGA SPI example source file.
 *
 *
 * @author
 *      Yuriy Kulikov yuriy.kulikov.87@gmail.com
 *
 *****************************************************************************/
// Compiler definitions include file
#include "avr_compiler.h"

// Scheduler include files
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Framework
extern "C" {
#include "strings.h"
}
#include "Serial.h"
#include "Spi.h"
// File headers
#include "SpiSlaveThread.h"
#include "Thread.h"

/**
 * @brief Starts SPI Slave example task
 * @param usartBuffer
 * @param debugLed
 * @param cPriority
 * @param taskHandle
 */
SpiSlaveThread::SpiSlaveThread(SpiSlave * spiSlave, Serial * serial, const char *name, unsigned short stackDepth, char priority)
:Thread(name, stackDepth, priority){
    this->spiSlave = spiSlave;
    this->serial = serial;
}
/**
 * @brief Slave task - simple spi-usart bridge
 * Tasks blocks on the queue, and as soon as data appears on the queue, task sends it
 * over the usart.
 * @attention use @link startSpiSlaveTask
 * @param pvParameters pass the struct using xTaskCreate
 */
void SpiSlaveThread::run() {
    uint8_t receivedChar='#';

    //Infinite loop
    for (;;) {
        //Function will block the task
        if (spiSlave->getByteFromQueue(&receivedChar, portMAX_DELAY) == pdPASS ) {
            serial->putPgmString(Strings_SpiSlaveExample1);
            serial->putInt(receivedChar,16);
            serial->putPgmString(Strings_newline);
            // report some kind of status
            spiSlave->status = receivedChar + 0x01;
        }
    }
}

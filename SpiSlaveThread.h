/* This file has been prepared for Doxygen automatic documentation generation.*/
/** @file *********************************************************************
 *
 * @brief  XMEGA SPI example header
 *
 *      This file contains task and function prototypes, definitions of structs,
 *      which are used to pass parameters to tasks.
 *
 * @par Documentation
 *     http://www.FreeRTOS.org - Documentation, latest information.
 *
 * @author
 *      Universität Erlangen-Nurnberg
 *		LS Informationstechnik (Kommunikationselektronik)
 *		Yuriy Kulikov
 *      Support email: Yuriy.Kulikov.87@googlemail.com
 *
 *****************************************************************************/

// Framework
#include "Serial.h"
#include "Spi.h"
#include "Thread.h"

/** Used to pass arguments to the task */
class SpiSlaveThread: public Thread {
private:
    /** Pointer to SpiSlave to use */
    SpiSlave * spiSlave;
    /** Pointer to USART buffer to use */
    Serial * serial;
public:
    SpiSlaveThread(SpiSlave * spiSlave, Serial * serial, const char *name, unsigned short stackDepth, char priority);
    void run();
};


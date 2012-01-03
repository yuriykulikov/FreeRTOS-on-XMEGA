/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA USART driver for FreeRTOS header file.
 *
 *      This file contains the function prototypes and enumerator definitions
 *      for various configuration parameters for the XMEGA USART driver.
 *
 *		Functions can be used with or without the kernel running. All operations are
 *		thread safe.
 *
 *      The driver is not intended for size and/or speed critical code, focusing
 *      on the ease of understanding and flexibility instead.
 *      This file is based on the driver provided by Atmel Corporation and drivers
 *      provided by FreeRTOS.org
 *
 * \author
 * Yuriy Kulikov
 *
 *****************************************************************************/
#ifndef USART_DRIVER_H
#define USART_DRIVER_H
#include "avr_compiler.h"
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

typedef enum {
	BAUD9600 = 1,
	BAUD19200 = 2,
} Baudrate;

/*! \brief Struct used when interrupt driven driver is used.
*  Struct containing pointer to a usart, a buffer and a location to store Data
*  register interrupt level temporary.
*/
class Serial {
private:
    /* \brief Pointer to USART module to use. */
    USART_t * module;
    /* \brief Data register empty interrupt level. */
    USART_DREINTLVL_t dreIntLevel;
    /** Default ticksToWait value */
    int ticksToWait;
    /* \brief Data buffer. */
    xQueueHandle RXqueue;
    xQueueHandle TXqueue;
public:
    signed char USART_RXComplete();
    signed char USART_DataRegEmpty();
    Serial(USART_t *module, Baudrate baudrate, uint8_t bufferSize, int ticksToWait);
    int8_t putByte(uint8_t data);
    int8_t putString(const char *string);
    int8_t putPgmString(const char *progmem_s);
    int8_t putInt(int16_t Int,int16_t radix);
    int8_t getByte(char * receivedChar);
    int8_t getByte(char * receivedChar, int ticks);
};
#endif


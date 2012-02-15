/* This file has been prepared for Doxygen automatic documentation generation.*/
/**@file *********************************************************************
 *
 * @brief  XMEGA SPI driver source file.
 *
 *      This file contains the source for the XMEGA SPI driver.
 *
 *      It is possible to have several devices on the same bus, using different
 *      SS pins.
 *
 *      Implementation relies on FreeRTOS and it is thread safe.
 *
 *		Bus contention prevention in master mode feature is not used.
 *
 *      Driver is not intended to be fast, especially in Slave mode. It
 *      concentrates the ease of understanding and use. Structures Slave and
 *      Master are not cast to void *, like it is done with xQueueHandle, for
 *      the reason to provide easier to understand code.
 *
 * @author
 *      Yuriy Kulikov yuriy.kulikov.87@gmail.com
 *
 *****************************************************************************/

// Scheduler include files.
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
extern "C" {
#include "port_driver.h"
}
#include "Spi.h"

/** Global pointer, used to pass data between task and ISR. If used, should point to the allocated Master structure */
SpiMaster *masterC;
/** Global pointer, used to pass data between task and ISR. If used, should point to the allocated Slave structure */
SpiSlave *slaveC;
/** Global pointer, used to pass data between task and ISR. If used, should point to the allocated Master structure */
SpiMaster *masterD;
/** Global pointer, used to pass data between task and ISR. If used, should point to the allocated Slave structure */
SpiSlave *slaveD;
//Prototype
signed char Spi_handleInterrupt (SPI_t *module, SpiMaster *master, SpiSlave *slave);
//Interrupt handling for the module on the port C
ISR(SPIC_INT_vect){
    //TODO declare ISR as naked, add context switch
    uint8_t isHigherPriorityTaskWoken = Spi_handleInterrupt(&SPIC, masterC, slaveC);
}
//Interrupt handling for the module on the port D
ISR(SPID_INT_vect){
    //TODO declare ISR as naked, add context switch
    uint8_t isHigherPriorityTaskWoken = Spi_handleInterrupt(&SPID, masterD, slaveD);
}

/**
 * @brief Common for all modules interrupt handler
 * @param module pointer to the module which has requested the interrupt
 * @param master pointer to the corresponding Master structure
 * @param slave pointer to the corresponding Slave structure
 * @return true if high priority task was unblocked by queue manipulation, false if not
 * */
inline signed char Spi_handleInterrupt (SPI_t *module, SpiMaster *master, SpiSlave *slave) {
	//Used to switch tasks, if queue or semaphore manipulation should wake a task.
	signed char isHigherPriorityTaskWoken = pdFALSE;
	//If spi module was initialized as master
	if (module->CTRL & SPI_MASTER_bm){
		//TODO for arrays
	}
	//If SPI module was initialized as slave
	else {
		// Get received data.
		uint8_t receivedChar = module->DATA;
		// Post the character on the queue of received characters.
		xQueueSendToBackFromISR(slave->commandsQueue, &receivedChar, &isHigherPriorityTaskWoken );
		// Put the status
		module->DATA = slave->status;
	}
	// If the post causes a task to wake force a context switch as the waken task
	// may have a higher priority than the task we have interrupted.
	return isHigherPriorityTaskWoken;
}

/**
 * @brief Initialize SPI slave on the specified module.
 * @param module pointer to the module to use
 * @param lsbFirst
 * @param mode
 * @param bufferSize
 * @return pointer to the Slave struct, to be used with functions prefixed by SpiSlave_
 */
SpiSlave::SpiSlave(SPI_t *module, bool lsbFirst, SPI_MODE_t mode, uint8_t queueSize) {
    // Create the queue
    commandsQueue = xQueueCreate( queueSize, sizeof(char));
    // Initial magic value, could be anything
    status = 0x42;
    //Switch by the pointer to the hardware module to set up port and ISR-Task communication structure
    if (module == &SPIC) {
        //Store the pointer to the object into the appropriate global variable
        slaveC = this;
        PORTC.DIRSET = SPI_MISO_bm;
    } else if (module == &SPID) {
        //Store the pointer to the object into the appropriate global variable
        slaveD = this;
        PORTD.DIRSET = SPI_MISO_bm;
    }
    // Set up SPI hardware module
    module->CTRL = SPI_ENABLE_bm | (lsbFirst ? SPI_DORD_bm : 0) | mode;
    module->INTCTRL = SPI_INTLVL_LO_gc;
}
/**
 * @brief Get single byte from the slave queue, which was put there when master performed write.
 * @param slave
 * @param receivedByte
 * @param ticksToWait
 * @return true if data was fetched from the queue, false otherwise
 */
char SpiSlave::getByteFromQueue(uint8_t *receivedByte, int ticksToWait) {
    return xQueueReceive(commandsQueue, receivedByte, ticksToWait);
}
/**
 * @brief This function initializes a SPI module as master.
 * @param module The SPI module.
 * @param port The I/O port where the SPI module is connected.
 * @param lsbFirst Data order will be LSB first if this is set to a non-zero value.
 * @param mode SPI mode (Clock polarity and phase).
 * @param intLevel SPI interrupt level.
 * @param clk2x SPI double speed mode
 * @param clockDivision SPI clock pre-scaler division factor.
 * @return pointer to the SpiMaster struct, to be used to create SpiDevice structures
 */
SpiMaster::SpiMaster(SPI_t *module, bool lsbFirst, SPI_MODE_t mode, bool clk2x, SPI_PRESCALER_t clockDivision){
    // Store module pointer first
    this->module=module;
    // Used to set pins as inputs or outputs
    PORT_t *port = NULL;
    //Switch by the pointer to the hardware module to set up port and ISR-Task communication structure
    if (module == &SPIC) {
        // SPIC interrupt vector will be triggered, connect it to the structure.
        masterC = this;
        port = &PORTC;
    } else if (module == &SPID) {
        // SPID interrupt vector will be triggered, connect it to the structure.
        masterD = this;
        port = &PORTD;
    }
    // Set MOSI and SCK pins outputs;
    port->DIRSET = SPI_MOSI_bm | SPI_SCK_bm;
    // SS should be set to the output, see SS pin section in the XMEGA SPI application note
    port->DIRSET = SPI_SS_bm;
    // Set SS pin high
    port->OUTSET = SPI_SS_bm;
    // Set MISO input
    port->DIRCLR = SPI_MISO_bm;
    //Create binary semaphore which will be used for synchronization between task and ISR
    //vSemaphoreCreateBinary(master->semaphore);
    // Create mutex which will be used to prevent using module by several tasks at the same time.
    mutex = xSemaphoreCreateMutex();
    // Set up the SPI hardware module
    module->CTRL = clockDivision|(clk2x ? SPI_CLK2X_bm : 0)|SPI_ENABLE_bm
            |(lsbFirst ? SPI_DORD_bm  : 0) |SPI_MASTER_bm |mode;	// TODO Add a field to hold the interrupt level which would be used to transmit large packets
    // Do not use interrupts for single-byte transmissions
    module->INTCTRL = SPI_INTLVL_OFF_gc;
}

/**
 * @brief Initialize device on the SPI bus controlled by master
 * @attention if you use the pin other than the standard SS pin, make sure the standard SS pin is not an input!
 * @param SpiMaster
 * @param ssPort
 * @param ssPinMask
 * @return pointer to the SpiDevice struct, to be used with functions prefixed by SpiMaster_
 */
SpiDevice::SpiDevice(SpiMaster *master, PORT_t *ssPort, uint8_t ssPinMask){
    // Store spi structure to use for this device
    this->master = master;
    // Store SS configuration.
    this->ssPort = ssPort;
    this->ssPinMask = ssPinMask;
    // Set SS pin output
    ssPort->DIRSET = ssPinMask;
    // Configure wired and pull-up
    PORT_ConfigurePins(ssPort, ssPinMask,false,false,PORT_OPC_WIREDANDPULL_gc,PORT_ISC_BOTHEDGES_gc );
    // Set SS pin high
    ssPort->OUTSET = ssPinMask;
}

/**
 * @brief Tries to obtain the mutex and pulls SS pin low
 * @param device
 * @param ticks To Wait to obtain the mutex
 * @return true if access to the bus was granted (mutex obtained), false otherwise
 */
char SpiDevice::startTransmission (int ticksToWait) {
    //Try to obtain mutex
    if (xSemaphoreTake(master->mutex, ticksToWait)==pdTRUE) {
        // Pull SS low
        ssPort->OUTCLR=ssPinMask;
        return pdTRUE;
    }
    // Function could not obtain the mutex, so return false
    return pdFALSE;
}
/**
 * @brief Set SS pin high and release the mutex
 * @param device
 */
void SpiDevice::stopTransmission () {
    // Pull SS high
    ssPort->OUTSET = ssPinMask;
    //Release the mutex
    xSemaphoreGive(master->mutex);
}

/**
 * @brief Shifts data with the slave device.
 * @attention Call SpiMaster_startTransmission() before and SpiMaster_stopTransmission() after
 * @param spiMaster
 * @param data
 * @return received value from slave
 */
uint8_t SpiDevice::shiftByte(uint8_t data) {
    //Put byte to initialize data transmission
    master->module->DATA=data;
    //Wait until byte is shifted
    while(!(master->module->STATUS & SPI_IF_bm)) { nop(); }
    // Accessing the DATA register will clear the SPI_IF_bm
    return master->module->DATA;
}

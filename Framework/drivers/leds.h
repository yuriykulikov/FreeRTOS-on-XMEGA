/*
 * ledGroup.h
 *
 *  Created on: 12.06.2011
 *      Author: Yuriy
 */

#ifndef LEDGROUP_H_
#define LEDGROUP_H_

#include "avr_compiler.h"
/* Group of leds */
class LedGroup {
private:
    uint8_t amountOfLeds;
    PORT_t ** ports;
    uint8_t *bitmasks;
public:
    LedGroup(uint8_t maxAmountOfLeds);
    void add(PORT_t * port, uint8_t bitmask, uint8_t isActiveLow);
    void set(uint8_t bitmask);
};

/*
 * Assume that RGB led is a led group of 3 leds,
 * then colors are bitmasks.
 */
typedef enum {
	RED = 0x01,
	GREEN = 0x02,
    BLUE= 0x04,

    ORANGE = 0x03,
    PINK = 0x05,
    SKY = 0x06,

    WHITE = 0x07,
	NONE = 0x00,
} Color_enum;

#endif /* LEDGROUP_H_ */

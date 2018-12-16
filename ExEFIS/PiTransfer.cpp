/*  Code for implementations of ByteTransfer class
 *
 *  Copyright 2018 Simon D. Levy - modified to use pigpio instead
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "PiTransfer.h"
#include <pigpio.h>



void PiI2C::begin(void)
{	
	_fd = i2cOpen(1, _address, 0);
}

void PiI2C::writeRegister(uint8_t subAddress, uint8_t data)
{
	i2cWriteByteData(_fd, subAddress, data);
}

void PiI2C::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    for (uint8_t i=0; i<count; ++i) 
    {
		dest[i] = i2cReadByteData(_fd, subAddress + i);
    }
}


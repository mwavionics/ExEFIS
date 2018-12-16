/*  Header for WiringPi implmentations of ByteTransfer class
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */
#pragma once
#include "ByteTransfer.h"

class PiI2C : public I2CTransfer {

public:

	PiI2C(uint8_t address)
		: I2CTransfer(address) {}

	void    begin(void);

	void    writeRegister(uint8_t subAddress, uint8_t data) override;
	void    readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) override;

private:

	int8_t _fd;
}
;

class PiSPI : public SPITransfer {

public:

	PiSPI(uint8_t bus, uint32_t speed) { _bus = bus; _speed = speed; }

	void    begin(void);

	void    writeRegister(uint8_t subAddress, uint8_t data) override;
	void    readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) override;

private:

	uint8_t  _bus;
	uint32_t _speed;

};
#include "hsc_pressure.h"
#include <unistd.h>
#include <stdio.h>
#include <pigpio.h>

#define AUX_SPI 256 //Set this to 0 for HW Rev 2

hsc_pressure::hsc_pressure()
{
	InitSPI();
	
}

hsc_pressure::hsc_pressure(int cs)
{
	chipEnable = cs;	
	InitSPI();
}

hsc_pressure::~hsc_pressure()
{
}

float hsc_pressure::getPressure(void)
{
	char vals[4] = { 0, 0, 0, 0 };
	char args[4] = { 0, 0, 0, 0 };
	SPITransfer(args, vals, 4);
	
	/* Check the MSBs of vals[0] for normal operation - should be zero*/
	float ret = 0.0f;
	int tics = 0;
	if (!(vals[0] & 0xC0))
	{
		tics = (vals[0] & 0x3F) * 256;
		tics += vals[1];
	
		ret = ((((float)tics - MIN_OUTPUT)*(maxPress - minPress)) /
			(MAX_OUTPUT - MIN_OUTPUT)) + minPress; 	
		
	}
	return (ret);
}
/**********************************************************************************************//**
* Module: hsc_pressure::InitSPI
***************************************************************************************************
* @brief	Setup the SPI

* @note		
*
* @todo		
*
* @param[in,out]	
* @param[in]		
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 11/06/17 - Created & Commented
**************************************************************************************************/
int hsc_pressure::InitSPI(void)
{	
	int flags = AUX_SPI; // 0b00000000000000000000000100000000;
	handle = spiOpen(chipEnable, 250000, flags);	
}


/**********************************************************************************************//**
* Module: hsc_pressure::SPITransfer
***************************************************************************************************
* @brief	

* @note 
*
* @todo 
*
* @param[in,out]	
* @param[in]		
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 11/06/17 - Created & Commented
**************************************************************************************************/
void hsc_pressure::SPITransfer(char* arg, char*resp, int length)
{	
	int val = spiRead(handle, resp, length);		
}



void hsc_pressure::set_params(int max, int min)
{
	maxPress = max;
	minPress = min;
}

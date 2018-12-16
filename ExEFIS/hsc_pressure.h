#pragma once

//#include <bcm2835.h>

#define MAX_COUNTS 16383 /* 100% of full output tics*/
#define MAX_OUTPUT 14745 /* 90% of full output tics of 16383*/
#define MIN_OUTPUT 1638	/*  10% of full output tics of 16383*/
//#define MAX_OUTPUT 16383
//#define MIN_OUTPUT 0
#define MAX_PRESSURE 15.0//101.325//30.5403f
#define MIN_PRESSURE 0.0f

class hsc_pressure
{
public:
	hsc_pressure();
	hsc_pressure(int cs);
	
	void set_params(int max, int min);
	
	~hsc_pressure();
	float getPressure();
	
protected:
	int InitSPI(void);
	void SPITransfer(char* arg, char*resp, int length);
	
private:
	int chipEnable = 0;
	int maxPress = MAX_PRESSURE;
	int minPress = MIN_PRESSURE;
	int handle;
};


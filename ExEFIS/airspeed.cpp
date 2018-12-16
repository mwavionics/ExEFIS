#include "airspeed.h"
#include <math.h>


static constexpr int speed[43] =
{ 
	15,
	 21,
	 25,
	 29,
	 33,
	 36,
	 39,
	 41,
	 44, 
	46,
	 49,
	 51,
	 53,
	 55,
	 57,
	 59,
	 62,
	 66,
	 69,
	 72,
	75,
	 78,
	 80,
	 83,
	 86,
	 88, 
	90, 
	93,
	 95,
	 97,
	 100, 
	102, 
	104,
	 109,
	 114,
	 118,
	 123, 
	127,
	131,
	 135,
	 139, 
	143
};
static constexpr int press[43] = 
{ 
	1, 
	2,
	 3,
	 4,
	 5, 
	6, 
	7,
	 8,
	 9,
	 10, 
	11,
	 12,
	 13,
	 14,
	 15, 
	16,
	 18,
	 20,
	 22,
	 24, 
	26,
	 28,
	 30,
	32,
	 34,
	 36,
	 38, 
	40, 
	42, 
	44,
	 46, 
	48, 
	50, 
	55,
	 60,
	 65,
	 70, 
	75, 
	80,
	 85,
	 90,
	 95
};

float airspeedavg[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int index = 0;

// 9/29/18 flight observations
/*
 *Kitfox    ExEFIS
 *91        80
 *69.5      63
 *99        87
 *
 **/
const float unitConversion = 237.334;
const float calibration = 1.077;
// 2  x  1 lbf     1 ft^3       12^2 in^2  1 slug ft    3600 sec    1 mile
// --------x--------------x-----------x----------x-----------x----------
// 1     in^2   0.0023769 slug   1 ft^2    1 lbf s^2      1 hr      5280 ft
float airspeed::getAirspeedMph(float pressurePsi, float tempC, float staticPressPSI)
{	
	float velocity = 0.0f;
	float press = pressurePsi;// - 0.001f;
	if (pressurePsi >= 0.0f)
	{		
		velocity = sqrt(press) * unitConversion * calibration;
	}
	
	
	airspeedavg[index] = velocity;
	index++;
	if (index >= 10) index = 0;
	
	float sum = 0;
	for (int i = 0; i < 10; i++)
	{
		sum += airspeedavg[i];
	}
	
	return sum/10.0f;
}

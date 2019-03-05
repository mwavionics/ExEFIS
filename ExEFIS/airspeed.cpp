#include "airspeed.h"
#include <math.h>

float airspeedavg[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int index = 0;


const float unitConversion = 237.334;
const float calibration = 1.077;
// 2  x  1 lbf     1 ft^3       12^2 in^2  1 slug ft    3600 sec    1 mile
// --------x--------------x-----------x----------x-----------x----------
// 1     in^2   0.0023769 slug   1 ft^2    1 lbf s^2      1 hr      5280 ft
float airspeed::getIASMph(float pressurePsi)
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

#include "altitude.h"
#include <math.h>


#if 0
//old calc
float altitude::getAltitudeFt(float pressurePSI, float setting)
{
	float mbars = 68.946f * pressurePSI;
	//altitude ft = ((mbars / 1013.25) ^ 0.190284) * 145366.45;
	float alt = (1-pow((mbars / 1013.25), 0.190284)) * 145366.45;
	alt = (setting - 29.92) * 1000 + alt;
	return (alt);
}
#endif

//Altitude = (10^(log(P/P_0)/5.2558797)-1/(-6.8755856*10^-6) 
//Where P = is atmospheric pressure at Altitude and 
//P_0 = the Sea Level pressure 
//Altitude is in feet 
//P & P_0 are unit independent and expressed as a ratio

float altitudeavg[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int aindex = 0;

float altitude::getAltitudeFt(float pressurePSI, volatile float setting)
{	
	float inHg = pressurePSI * 2.03602;	
	volatile double llog = log10(inHg / setting) ;
	double top = pow(10.0f, (llog / 5.2558797f)) - 1.0f;
	double bot = -6.8755856*pow(10.0f, -6);	
	float alt = top / bot;
	
	altitudeavg[aindex] = alt;
	aindex++;
	if (aindex >= 10)aindex = 0;
	float sum = 0;
	for (int i = 0; i < 10; i++)
	{
		sum += altitudeavg[i];
	}
	
	return (sum / 10.0f);
}


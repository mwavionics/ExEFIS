#include "knobs.h"
#include "RotaryEncoder.h"



knobs::knobs()
{	
	//NOTE: These are switched for 3.5 version as configured.
	//For the PiZero ADB, this is pin 12 for the button
	//left = new RotaryEncoder(12, 13, 19, 0);
	right = new RotaryEncoder(5, 6, 22, 1);

	
}


knobs::~knobs()
{
}

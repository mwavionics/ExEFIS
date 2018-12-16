#pragma once
#include "RotaryEncoder.h"

class knobs
{
public:
	knobs();
	~knobs();
	
	RotaryEncoder *left;
	RotaryEncoder *right;
};


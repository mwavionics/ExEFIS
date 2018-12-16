#pragma once
#include <unistd.h>


/*
 *Class for the Rotory Encoder for the front of the device
 *Natively, this class supports 2 encoders even though the 3.5 only has 1
 **/
class RotaryEncoder
{
public:
	
	static int encoderCount;
	static RotaryEncoder* encoders[2];
	static void eventHandler(int gpio, int level, unsigned tick);
	static void pressHandler(int gpio, int level, unsigned tick);
	
	RotaryEncoder(int a, int b, int c, int index);
	~RotaryEncoder();
	int getValue(void);
	void setValue(int val);
	int getPress(bool clear);
	bool getSinglePress(void);
	
protected:
	

	int value = 0;
	int press = 0;
	
private:
	int pin_a;
	int pin_b;
	int pin_c;
	
	int lastEncoded;
	int lastPress;
	char lastPressState;
	
};


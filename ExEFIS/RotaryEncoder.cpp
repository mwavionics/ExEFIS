#include "RotaryEncoder.h"
//#include <QDebug>
#include <pigpio.h>
#include "pigpiod_if.h"


int RotaryEncoder::encoderCount = 2;
RotaryEncoder* RotaryEncoder::encoders[2] = { NULL, NULL };


RotaryEncoder::RotaryEncoder(int a, int b, int c, int index)
{	
	pin_a = a;
	pin_b = b;
	pin_c = c;	
	
	
	//pinMode(pin_a, INPUT);
	//pinMode(pin_b, INPUT);
	//pinMode(pin_c, INPUT);
	
	//pullUpDnControl(pin_a, PUD_UP);
	//pullUpDnControl(pin_b, PUD_UP);
	//pullUpDnControl(pin_c, PUD_UP);
	
//	wiringPiISR(pin_a, INT_EDGE_BOTH, RotaryEncoder::eventHandler);
//	wiringPiISR(pin_b, INT_EDGE_BOTH, RotaryEncoder::eventHandler);
//	wiringPiISR(pin_c, INT_EDGE_BOTH, RotaryEncoder::pressHandler);

	
	//bcm2835_gpio_len(pin_a);
	//bcm2835_gpio_hen(pin_a);
	//bcm2835_gpio_len(pin_b);
	//bcm2835_gpio_hen(pin_b);
	//bcm2835_gpio_len(pin_c);
	//bcm2835_gpio_hen(pin_c);
	
	int ret = gpioSetISRFunc((unsigned)pin_a, EITHER_EDGE, 0, (gpioISRFunc_t)RotaryEncoder::eventHandler);
	ret = gpioSetISRFunc(pin_b, EITHER_EDGE, 0, (gpioISRFunc_t)RotaryEncoder::eventHandler);
	ret = gpioSetISRFunc(pin_c, EITHER_EDGE, 0, (gpioISRFunc_t)RotaryEncoder::pressHandler);
	
	encoders[index] = this;
}


RotaryEncoder::~RotaryEncoder()
{
}


void RotaryEncoder::eventHandler(int gpio, int level, unsigned tick)
{
	for (int i = 0; i < RotaryEncoder::encoderCount; i++)
	{
		RotaryEncoder* enc = encoders[i];
		
		if (enc != NULL)
		{			
			char MSB = gpioRead(enc->pin_a);
			char LSB = gpioRead(enc->pin_b);

			int encoded = (MSB << 1) | LSB;
			int sum = (enc->lastEncoded << 2) | encoded;
				
			/* need to throw out the encoder ticks that land between detents on the knob...*/
			/* sum == 13 || sum == 11 value ++*/
			if (sum == 0b1101 || /*sum == 0b0100 || sum == 0b0010 || */ sum == 0b1011) enc->value++;
			/* sum == 14 || sum == 7 value -- */
			if (sum == 0b1110 ||  sum == 0b0111 /* || sum == 0b0001 || sum == 0b1000*/) enc->value--;

		//	printf("sum %d value %d \n", sum, enc->value);	
			enc->lastEncoded = encoded;	
		}
	}
}

void RotaryEncoder::pressHandler(int gpio, int level, unsigned tick)
{
	for (int i = 0; i < RotaryEncoder::encoderCount; i++)
	{
		RotaryEncoder* enc = encoders[i];
		if (enc != NULL)
		{	
		
			char cur;
		
			cur = gpioRead(enc->pin_c);
			if (cur != enc->lastPressState && !cur)
			{
				enc->press++;
			}
			enc->lastPressState = cur;
		}
	}
}


int RotaryEncoder::getValue(void)
{
	return (value);
}


void RotaryEncoder::setValue(int val)
{
	value = val;
}

bool RotaryEncoder::getSinglePress(void)
{
	bool pushOccurred = false;
	if (press != lastPress) pushOccurred = true;
	lastPress = press;
	return pushOccurred;
}


int RotaryEncoder::getPress(bool clear)
{
	int ret = press;
	if (clear)
	{
		press = 0;
		lastPress = press;
	}
	return (ret);
}

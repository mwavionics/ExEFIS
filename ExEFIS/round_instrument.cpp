#include "round_instrument.h"
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <math.h>



round_instrument::round_instrument(QWidget *parent)
	: QWidget(parent)
{
	setAttribute(Qt::WA_NoSystemBackground);
	setAttribute(Qt::WA_TransparentForMouseEvents);
	this->penColor = Qt::black;
	
	blinkTimer = new QTimer(this);
	blinkTimer->setInterval(500);
	connect(blinkTimer, SIGNAL(timeout()), this, SLOT(onBlinkTimer()));
	
}

round_instrument::round_instrument(QWidget *parent, QColor c)
	: QWidget(parent)
{
	setAttribute(Qt::WA_NoSystemBackground);
	setAttribute(Qt::WA_TransparentForMouseEvents);
	this->penColor = c;
	
	blinkTimer = new QTimer(this);
	blinkTimer->setInterval(500);
	connect(blinkTimer, SIGNAL(timeout()), this, SLOT(onBlinkTimer()));
}

void round_instrument::setValue(int val)
{
	value = val;
	update();
}

void round_instrument::setupInstrument(int* vals, int numVals)
{
	numValues = numVals <= 30 ? numVals : 30;
	for (int i = 0; i < numVals; i++)
	{
		values[i] = vals[i];
	}
}

void round_instrument::paintEvent(QPaintEvent * /* event */)
{	
	/* Bound the DG to min and max values with a wrap*/
	if (value > 360.0f) value = 360.0f;
	if (value <= 0.0f) value = 360.0f;
	/* Offset the value with the setting*/
	float val = value + setting;
	if (val <= 0) val += 360.0f;
	if (val > 360.0f) val -= 360.0f;
	val /= 10.0f;
	
	/* Calculate the size of the rectangle for the whole instrument*/
	int tall = height() * 0.7f;
	int wide = width() * 0.7f;	
	int y = height() * 0.15f;
	int x = (width()/2) - (tall/2);
	int xcenter = width() / 2;
	int ycenter = height() / 2;
	/* This is the rectangle for the outline of the instrument*/
	QRect rect(x, y, tall, tall);

	
	/* Paint the outline*/
	QPainter painter(this);
	painter.setPen(penColor);
	pen.setWidth(2);
	pen.setColor(Qt::black);
	painter.setPen(pen);
	painter.setBrush(QColor::fromRgb(169, 169, 169, 100));
	painter.drawEllipse(rect);
			
	/* find the value above the indicated value*/
	bool found = false;
	int i;
	for (i = 0; i < numValues && !found; i++)
	{
		if (values[i] > val) found = true;
	}
	
	i -= 2;
	
	if (i < 0)i = 0;
	int abovevalue = values[i];
	int belowvalue = values[i - 1];		
	float shift = 0.0f;
	if (abovevalue - belowvalue < 0) 
	{
		abovevalue += 36;
		float newval = val;
		if (newval < belowvalue) newval += 36;
		shift = (360.0f *
			(newval - belowvalue)) / (abovevalue - belowvalue);
		shift *= -1.0f;   //shift opposite direction;
	}
	else
	{
		shift = (360.0f *
			(val - belowvalue)) / (abovevalue - belowvalue);
		shift *= -1.0f;    //shift opposite direction;
	}	

	
	QPainter nums(this);
	pen.setWidth(3);
	pen.setColor(Qt::black);
	nums.setPen(pen);
	nums.translate(xcenter, ycenter);
	double newRadius = (tall - 20) / 2;
	float anglestep = 360.0f / number_divs;
	float start = -90.0f - val * 10.0f;
	for (int t = 1; t < number_divs+1; t++)
	{
		nums.save();
		nums.rotate(start + (t*anglestep));
		nums.drawLine(newRadius - 8, 0, newRadius, 0);
		nums.rotate(anglestep *.1);
		nums.drawText(newRadius - 25, 0, QString::number(values[t-1]));
		nums.restore();
	}

	
	/* Create the center rectangle around the real value*/
	/* Top left corner of the center box to paint the main value*/
	QPoint centerTLeft(xcenter - (tall / 8), y + (tall / 2) - (tall / 16));
	QPoint centerBRight(xcenter + (tall / 8), y + (tall / 2) + (tall / 16));		
	QRect centerRect(centerTLeft, centerBRight);
	
	QPainter center(this);
	pen.setWidth(2);
	pen.setColor(Qt::black);
	center.setPen(pen);
	center.setBrush(QColor::fromRgb(0, 0, 0, 150));
	
	if (!this->blinking) pen.setColor(Qt::black);
	else pen.setColor(Qt::blue);
	center.setPen(pen);
	center.drawRect(centerRect);
	center.setPen(Qt::white);
	center.drawText(centerRect, Qt::AlignCenter, QString::number(val * 10));	
	pen.setWidth(1);
}

void round_instrument::change(void)
{
	update();
}


round_instrument::~round_instrument()
{
}


void round_instrument::onBlinkTimer(void)
{
	blinking = !blinking;
}


void round_instrument::setSetting(float stg)
{
	if (this->editMode)
	{	
		this->setting = stg;
		if (setting > 360.0f) setting -= 360.0f;
		if (setting < 0.0f) setting += 360.0f; 
		update();
	}
}


void round_instrument::setEditMode(bool emode)
{
	
	this->editMode = emode;
	if (this->editMode) blinkTimer->start();
	else 
	{
		blinkTimer->stop();
		blinking = false;
	}
	
}


void round_instrument::toggleEditMode(void)
{
	setEditMode(!editMode);
	if (!editMode) update();
}


float round_instrument::getSetting(void)
{
	return (setting);
}

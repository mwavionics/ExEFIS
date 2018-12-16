#include "vertical_instrument.h"
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <math.h>


vertical_instrument::vertical_instrument(QWidget *parent)
	: QWidget(parent)
{
	setAttribute(Qt::WA_NoSystemBackground);
	setAttribute(Qt::WA_TransparentForMouseEvents);
	this->penColor = Qt::red;
	
	blinkTimer = new QTimer(this);
	blinkTimer->setInterval(500);
	connect(blinkTimer, SIGNAL(timeout()), this, SLOT(onBlinkTimer()));
	
}

vertical_instrument::vertical_instrument(QWidget *parent, QColor c)
	: QWidget(parent)
{
	setAttribute(Qt::WA_NoSystemBackground);
	setAttribute(Qt::WA_TransparentForMouseEvents);
	this->penColor = c;
	
	blinkTimer = new QTimer(this);
	blinkTimer->setInterval(500);
	connect(blinkTimer, SIGNAL(timeout()), this, SLOT(onBlinkTimer()));
}

void vertical_instrument::setValue(int val)
{
	value = val;
	update();
}

void vertical_instrument::setupInstrument(int* vals, int numVals)
{
	numValues = numVals <= 30 ? numVals : 30;
	for (int i = 0; i < numVals; i++)
	{
		values[i] = vals[i];
	}
}

void vertical_instrument::paintEvent(QPaintEvent * /* event */)
{	
	/* Calculate the size of the rectangle for the whole instrument*/
	int tall = height() * 0.8f;
	int wide = width() * 0.8f;	
	int y = height() * 0.1f;
	int x = width() * 0.1f;
	/* This is the rectangle for the outline of the instrument*/
	QRect rect(x, y, wide, tall);
	
	/* Paint the outline*/
	QPainter painter(this);
	painter.setPen(penColor);
	painter.setBrush(QColor::fromRgb(169, 169, 169, 100));
	painter.drawRect(rect);
	
	/* vert is the height of each rectangle for values*/
	int vert = tall / vertical_divs / 2;
	bool found = false;
	int i;
	/* find the value above the indicated value*/
	for (i = 0; i < numValues && !found; i++)
	{
		if (values[i] > value) found = true;
	}
	
	i -= 2;
	
	if (i < 0)i = 0;
	int abovevalue = values[i];
	int belowvalue = values[i-1];

	
	
	/* first idx is the top value in the values list*/
	int firstidx = i + (vertical_divs / 2); // - 1;
	int lastidx = firstidx - vertical_divs;
	int verticalDistance = vert * 2; /* dist between them*/
	int shift = (verticalDistance *
		(value - abovevalue)) / (abovevalue - belowvalue);
	
	
	/* Create the rectangles for each of the values*/
	QRect rects[vertical_divs];
	for (int j = 0; j < vertical_divs; j++)
	{
		rects[j] = QRect(x, y + (vert * 2*j) + shift, wide, vert);
	}
	
	/* Create the center rectangle around the real value*/
	/* Top left corner of the center box to paint the main value*/
	QPoint centerTLeft(x, y + (tall / 2) - (vert / 2));
	QPoint centerBRight(x + wide, y + (tall / 2) + (vert / 2));		
	QRect centerRect(centerTLeft, centerBRight);
	
	QPoint bottomTLeft(x, y + tall - vert);
	QPoint bottomBRight(x + wide, y + tall);
	QRect bottomRect(bottomTLeft, bottomBRight);
	

	for (int j = 0; j < vertical_divs; j++)
	{
		/* First index is the highest value of i becuase it reads top down high to low*/
		painter.drawText(rects[j], Qt::AlignLeft, ((firstidx - j) > 0) ? QString::number(values[firstidx - j]) : " ");
	}
	
	painter.setPen(Qt::black);
	painter.setBrush(QColor::fromRgb(0, 0, 0, 150));
	painter.drawRect(centerRect);
	painter.setPen(Qt::white);
	painter.drawText(centerRect, Qt::AlignCenter, QString::number(value));
	
	if (showSetting)
	{
		pen.setWidth(2);
		if (!this->blinking)	pen.setColor(Qt::black);
		else pen.setColor(Qt::blue);
		painter.setPen(pen);
		painter.setBrush(QColor::fromRgb(0, 0, 0, 150));
		painter.drawRect(bottomRect);
		painter.setPen(Qt::white);
		painter.drawText(bottomRect, Qt::AlignCenter, QString::number(setting / pow(10, settingPrec), 'f', settingPrec));
		pen.setWidth(1);
	}
}

void vertical_instrument::change(void)
{
	update();
}


vertical_instrument::~vertical_instrument()
{
}


void vertical_instrument::onBlinkTimer(void)
{
	blinking = !blinking;
}


void vertical_instrument::setSetting(float stg)
{
	if (this->editMode)
	{
		this->setting = stg;
	}
}


void vertical_instrument::setEditMode(bool emode)
{
	if (showSetting)
	{
		this->editMode = emode;
		if (this->editMode) blinkTimer->start();
		else 
		{
			blinkTimer->stop();
			blinking = false;
		}
	}
}


void vertical_instrument::toggleEditMode(void)
{
	setEditMode(!editMode);
}


int vertical_instrument::getValue(void)
{
	return (value);
}

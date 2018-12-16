#include "directional_gyro.h"
#include "QDebug"



directional_gyro::directional_gyro(QWidget *parent)
	: QWidget(parent)
{
	setAttribute(Qt::WA_NoSystemBackground);
	setAttribute(Qt::WA_TransparentForMouseEvents);
	this->penColor = Qt::white;
	value = -1.0f;
	setting = 0.0f;
	
	blinkTimer = new QTimer(this);
	blinkTimer->setInterval(500);
	connect(blinkTimer, SIGNAL(timeout()), this, SLOT(onBlinkTimer()));
}


directional_gyro::~directional_gyro()
{
}

void directional_gyro::paintEvent(QPaintEvent *event)
{
	/* assign the values and cap*/
	if(value > 360.0f) value = 360.0f;
	if(value <= 0.0f) value = 360.0f;
	float val = value - setting;
	if (val <= 0) val += 360.0f;
	val	/= 10.0f;	
	
	/* grab the width and height of the drawing area*/
	int wide = width() * 0.75f;
	int high = height() * 0.9f;
	/* grab the width and heigh of each drawing box for the nubmers*/
	int y = height() * 0.4f;
	int x = width() * 0.125f;
	
	/* this is the width of the indicator*/
	int wr = width() / 50;
	if (wr < 2) wr = 2;
	
	/* top left of the indicator, bottom right of the indicator, then the rectangle*/
	QPoint tl = QPoint((width() / 2) - (wr / 2), height() * 0.1f);
	QPoint br = QPoint((width() / 2) + (wr / 2), high);
	QRect lne = QRect(tl, br);
	
	/* Paint on the indicator*/
	QPainter painter(this);
	painter.setPen(penColor);
	if (blinking)painter.setPen(Qt::blue);
	painter.setBrush(Qt::white);
	painter.drawRect(lne);
	
	/* Setup the pen to paint the numbers*/
	painter.setPen(Qt::black);
	painter.setBrush(Qt::black);
	
	/* this is more of a const, for use in number of values across*/
	int horiz_divs = 4;
	
	/* horiz is the width of each rectangle for values*/
	int horiz = wide / horiz_divs; // / 2; //"4" is the number of divs
	
	bool found = false;
	int i = 0;
	int j = 0;
	/* find the value above the indicated value*/
	for (i = 0; i < 12 && !found; i++)
	{
		if (values[i] > val)
		{	
			found = true;
			j = i;
		}			
	}
	
	if (!found) j = 12;
	//values[i] goes to the right of the indicator
	//values[i-1] goes on or to the left of the indicator
	i = j - 1;
	if (i < 0) i += 12;
	if (j > 11) j -= 12;
	
	volatile int abovevalue = values[j];/* Goes to the right of the indicator*/
	volatile int belowvalue = values[i]; /* goes to the left of the indicator */
	///i-1 < 0 ? values[12+i-1] : values[i - 1];
		
	/* first idx is the left value in the values list*/
	volatile int firstidx = i - (horiz_divs / 2) - 1;		
	if (firstidx < 0) firstidx += 12;	
	
	int horizDistance = horiz * 2; /* dist between them*/
	
	//int divisor = belowvalue < abovevalue ? abovevalue - belowvalue : abovevalue + (belowvalue - 36);
	float shift = 0.0f;
	if(abovevalue - belowvalue < 0) 
	{
		abovevalue += 36;
		float newval = val;
		if (newval < belowvalue) newval += 36;
		shift = (horizDistance *
			(newval - belowvalue)) / (abovevalue - belowvalue);
		shift *= -1.0f;  //shift opposite direction;
	}
	else
	{
		shift = (horizDistance *
			(val - belowvalue)) / (abovevalue - belowvalue);
		shift *= -1.0f;   //shift opposite direction;
	}

	
	/* Create the rectangles for each of the values*/
	QRect rects[horiz_divs + 2];
	for (int k = 0; k < horiz_divs + 2 ; k++)
	{
		rects[k] = QRect(x + (horiz * 2*(k-1)) + shift, y, high, horiz);
	}	

	for (int k = 0; k < horiz_divs + 2; k++)
	{
		volatile int vIndex = (firstidx + k + 1);
		if (vIndex > 11) vIndex -= 12; 		
		painter.drawText(rects[k], Qt::AlignLeft, QString::number(values[vIndex]));		
	}
}

#include <QWidget>
#include <QPainter>
#include <QPen>
#include "slipskid_instrument.h"



slipskid_instrument::slipskid_instrument(QWidget *parent)
	: QWidget(parent)
{
	setAttribute(Qt::WA_NoSystemBackground);
	setAttribute(Qt::WA_TransparentForMouseEvents);
	this->penColor = Qt::white;
	value = 0.0f;
	for (int i = 0; i < NUM_SS_VALS; i++)
	{
		vals[i] = 0;
	}
	index = 0;
}


slipskid_instrument::~slipskid_instrument()
{
}


void slipskid_instrument::setValue(float val)
{
	vals[index] = val;
	index++;
	if (index >= NUM_SS_VALS) index = 0;
	float sum = 0.0f;
	for (int i = 0; i < NUM_SS_VALS; i++)
	{
		sum += vals[i];
	}
	value = sum/10.0f;
	update();
}


void slipskid_instrument::paintEvent(QPaintEvent *event)
{
	int wide = width() / 10;
	int hi = wide;
	int tlx = (width() / 2) - (wide / 2); /* Top Left X */
	int tly = (height() / 2) - (hi / 2); /* Top Left Y */
	
	QRect r = QRect(tlx, tly, wide, hi);
	
	int wr = wide / 5;
	QRect lv = QRect(tlx - (2*wr), tly, wr, hi);
	QRect lr = QRect(((tlx + wide) + (wr)), tly, wr, hi);
	
	QPainter painter(this);
	painter.setPen(penColor);
	painter.setBrush(Qt::black);
	
	painter.translate(-value*wide, 0);
	painter.drawEllipse(r);
	
	painter.translate(value*wide, 0);
	painter.setPen(Qt::black);
	painter.drawRect(lv);
	painter.drawRect(lr);
}

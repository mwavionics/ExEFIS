#include "horizon_instrument.h"
#include <QWidget>
#include <QPainter>
#include <QBrush>
#include <math.h>


horizon_instrument::horizon_instrument(QWidget *parent)
	: QWidget(parent)
{
	_angleMutex = new QMutex();
	QMutexLocker *l = new QMutexLocker(_angleMutex);
	_angle = 0;
	_azimuth = 0;
	earthColor = QColor::fromRgb(114, 59, 34, 255);
	skyColor = QColor::fromRgb(125, 206, 250, 255);
	verticalOffset = 0;
}

horizon_instrument::horizon_instrument(QWidget *parent, QColor earth, QColor sky, float vOffset)
	:horizon_instrument(parent)
{
	earthColor = earth;
	skyColor = sky;
	setVerticalOffset(vOffset);
}

void horizon_instrument::paintEvent(QPaintEvent *event)
{	
	QPainter painter(this);
	
	/*Move the Origin to the center so it will rotate*/
	float azpix = height() / 60.0f;
	painter.translate(width() / 2, (height() /2 ) + (azpix * _azimuth) + (azpix * verticalOffset));//(height() / 2) + _azimuth));
	painter.rotate(_angle);

	/* Set the top half of the horizon */	
	QRect top = QRect(-3*width() / 2, -4*height() / 2, 3*width(), 4*height() / 2); 
	
	/* Set the bottom half of the horizon */
	QRect bot = QRect(-3*width() / 2, 0, 3*width(), 4*height() / 2);
	
	
	QPoint l1 = QPoint(-(width() / 10), 0);
	QPoint r1 = QPoint((width() / 10), 0);
	
	/* Paint the Sky and Earth */
	painter.setBrush(earthColor);
	painter.setPen(Qt::NoPen);	
	painter.drawRect(bot);
	
	painter.setBrush(skyColor);
	painter.setPen(Qt::NoPen);
	painter.drawRect(top);		

	painter.setBrush(Qt::black);
	QPen p = QPen();
	p.setColor(Qt::black);
	p.setWidth(height() / 200);
	p.setStyle(Qt::SolidLine);
	painter.setPen(p);
	
	
	//painter.drawLine(l1, r1);
	painter.drawLine(l1.x(), l1.y() + (azpix * 10), r1.x(), r1.y() + (azpix * 10));
	painter.drawLine(l1.x(), l1.y() - (azpix * 10), r1.x(), r1.y() - (azpix * 10));
	painter.drawLine(l1.x(), l1.y() + (azpix * 20), r1.x(), r1.y() + (azpix * 20));
	painter.drawLine(l1.x(), l1.y() - (azpix * 20), r1.x(), r1.y() - (azpix * 20));

	
}

void horizon_instrument::setAngle(qreal angle)
{
	_angle = angle;
	update();
}

void horizon_instrument::setAzimuth(qreal azimuth)
{
	_azimuth = azimuth;
}

horizon_instrument::~horizon_instrument()
{
}

void horizon_instrument::setVerticalOffset(float deg)
{
	if (deg < 20.0f && deg > -20.0f) verticalOffset = deg;
}

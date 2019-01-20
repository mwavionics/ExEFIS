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
	this->penColor = Qt::black;
	_angle = 0;
	_azimuth = 0;
}

horizon_instrument::horizon_instrument(QWidget *parent, QColor c)
{
	_angleMutex = new QMutex();
	QMutexLocker *l = new QMutexLocker(_angleMutex);
	this->penColor = c;
	_angle = 0;
	_azimuth = 0;
}

void horizon_instrument::paintEvent(QPaintEvent *event)
{	
	QPainter painter(this);
	
	/*Move the Origin to the center so it will rotate*/
	float azpix = height() / 60.0f;
	painter.translate(width() / 2, (height() /2 ) + (azpix * _azimuth));//(height() / 2) + _azimuth));
	painter.rotate(_angle);

	/* Set the top half of the horizon */	
	QRect top = QRect(-1.5*width() / 2, -2*height() / 2, 1.5*width(), 2*height() / 2);
	QColor topcolor = QColor::fromRgb(125, 206, 250, 255);  //Sky Blue		
	
	/* Set the bottom half of the horizon */
	QRect bot = QRect(-1.5*width() / 2, 0, 1.5*width(), 2*height() / 2);
	QColor botcolor = QColor::fromRgb(114, 59, 34, 255);  	//Earth Browm
	
	QPoint center = QPoint(width() / 2, height() / 2);
	
	
	/* Paint the Sky and Earth */
	painter.setBrush(botcolor);
	painter.setPen(Qt::NoPen);	
	painter.drawRect(bot);
	
	painter.setBrush(topcolor);
	painter.setPen(Qt::NoPen);
	painter.drawRect(top);	
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



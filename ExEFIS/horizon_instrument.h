#pragma once

#include "horizon_instrument.h"
#include <QWidget>
#include <QPen>
#include <QMutex>

class horizon_instrument :public QWidget
{
	Q_OBJECT
		
public :
	horizon_instrument(QWidget *parent = 0);
	horizon_instrument(QWidget *parent, QColor earth, QColor sky, float vOffset);
	void setAngle(qreal angle);
	void setAzimuth(qreal azimuth);
	void setVerticalOffset(float deg);
	float verticalOffset;
	~horizon_instrument();
	
protected: 
	void paintEvent(QPaintEvent *event) override;
	
private:
	
	qreal _angle;
	qreal _azimuth;
	QMutex *_angleMutex;
	QColor earthColor;
	QColor skyColor;
};


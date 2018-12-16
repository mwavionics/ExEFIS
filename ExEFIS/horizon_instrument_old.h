#pragma once

#include "horizon_instrument.h"
#include <QWidget.h>
#include <QPen.h>
#include <QMutex>

class horizon_instrument :public QWidget
{
	Q_OBJECT
		
public :
	horizon_instrument(QWidget *parent = 0);
	horizon_instrument(QWidget *parent, QColor c);
	void setAngle(qreal angle);
	void setAzimuth(qreal azimuth);
	~horizon_instrument();
	
protected: 
	void paintEvent(QPaintEvent *event) override;
	
private:
	QPen pen;
	QColor penColor;
	qreal _angle;
	qreal _azimuth;
	QMutex *_angleMutex;
};


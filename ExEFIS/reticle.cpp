#include "reticle.h"
#include <QWidget>
#include <QPoint>
#include <QPainter>


reticle::reticle(QWidget *parent)
	: QWidget(parent)
{
	wingsLevel = false;
	setting = 0;
	blinking = false;
	
	blinkTimer = new QTimer(this);
	blinkTimer->setInterval(500);
	connect(blinkTimer, SIGNAL(timeout()), this, SLOT(onBlinkTimer()));
}


reticle::~reticle()
{
}

void reticle::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	
	/* Draw the reticle */
	QPoint points[4];
	
	points[0] = QPoint(width() / 2, height() / 2);
	points[2] = points[0];
	int botmiddleoffset = height() / 20;
	points[2].setY(points[0].y() + botmiddleoffset);
	points[1] = QPoint(points[0].x() - (width() / 4), points[0].y() + (width() / 8));
	points[3] = QPoint(points[0].x() + (width() / 4), points[0].y() + (width() / 8));
	
	painter.setBrush(Qt::white);
	if (wingsLevel)	painter.setPen(Qt::black);
	else painter.setPen(Qt::red);
	
	if (blinking) painter.setPen(Qt::blue);
	
	painter.drawPolygon(points, 4);
	/* Draw the horizontal bars */
	
	int ho = height() / 100;
	int wo = width() / 8;
	
	QRect left = QRect(width() / 10, (height() / 2) - ho, wo, 2*ho);
	QRect right = QRect((width() - (width() / 10) - wo), height() / 2 - ho, wo, 2*ho);
	
	painter.drawRect(left);
	painter.drawRect(right);
	
}

void reticle::setWingsLevel(bool level)
{
	wingsLevel = level;
}

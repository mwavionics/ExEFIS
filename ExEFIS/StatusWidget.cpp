#include <QRect>
#include <QPainter>
#include "StatusWidget.h"



StatusWidget::StatusWidget(QWidget *parent)
{
	status = "";
}


StatusWidget::~StatusWidget()
{
	status = "";
}


void StatusWidget::paintEvent(QPaintEvent *event)
{
	if (show)
	{	
		QPainter painter(this);
		painter.setPen(Qt::black);
		
		QRect rec = QRect(0, 0, width(), height());
		painter.drawRect(rec);
		if (value == 0)
		{
			painter.drawText(rec, Qt::AlignLeft, "Static DG");
		}
		if (value == 1)
		{
			painter.drawText(rec, Qt::AlignLeft, "Live DG");
		}
		if (value == 2 || value == 3)
		{
			painter.drawText(rec, Qt::AlignLeft, status);
		}	
		if (value == 4)
		{
			painter.drawText(rec, Qt::AlignLeft, "BRIGHT");
		}
		if (value == 5)
		{
			painter.drawText(rec, Qt::AlignLeft, "MED");
		}
		if (value == 6)
		{
			painter.drawText(rec, Qt::AlignLeft, "DIMMEST");
		}
	}
	else 
	{
		if (value ==0 || value == 2 || value == 3)
		{
			QPainter painter(this);
			painter.setPen(Qt::black);
			
			QRect rec = QRect(0, 0, width(), height());
			painter.drawText(rec, Qt::AlignLeft, status);
		}
	}	
}


void StatusWidget::setValue(int val)
{
	value = val;
	if (value >= 7) value = 0;
	if (value < 0) value = 6;
}

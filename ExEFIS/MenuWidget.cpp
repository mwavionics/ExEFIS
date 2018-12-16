#include <QRect>
#include <QPainter>
#include "MenuWidget.h"



MenuWidget::MenuWidget(QWidget *parent)
{
	status = "";
	numItems = 7;
	items[0].str = "Brightness BRIGHT";
	items[0].alwaysShow = false;
	items[1].str = "Brightness MED";
	items[1].alwaysShow = false;
	items[2].str = "Brightness DIM";
	items[2].alwaysShow = false;
	items[3].str = "Item 3";
	items[3].alwaysShow = false;
	items[4].str = "Item 4";
	items[4].alwaysShow = true;
	items[5].str = "Item 5";
	items[5].alwaysShow = true;
	items[7].str = "Exit App";
	items[7].alwaysShow = false;
}


MenuWidget::~MenuWidget()
{
	status = "";
}


void MenuWidget::paintEvent(QPaintEvent *event)
{
	if (show)
	{	
		QPainter painter(this);
		painter.setPen(Qt::black);
		
		QRect rec = QRect(0, 0, width(), height());
		painter.drawRect(rec);
		painter.drawText(rec, Qt::AlignLeft, items[index].str);	
	}
	else 
	{
		if (items[index].alwaysShow)
		{
			QPainter painter(this);
			painter.setPen(Qt::black);
			
			QRect rec = QRect(0, 0, width(), height());
			painter.drawText(rec, Qt::AlignLeft, items[index].str);
		}
	}	
}

void MenuWidget::setStatus(int arg1, const QString &arg2)
{
	if (arg1 < numItems)
	{
		items[arg1].str = arg2;
	}
}

void MenuWidget::setValue(int val)
{
	value = val;
	if (value >= numItems) value = 0;
	if (value < 0) value = (numItems-1);
	index = value;
}

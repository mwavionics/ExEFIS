#pragma once

#include <QWidget>
#include <QPen>

class reticle :public QWidget
{
	
	Q_OBJECT
	
public :
	reticle(QWidget *parent = 0);
	
	~reticle();
	
protected:
	void paintEvent(QPaintEvent *event) override;
};


#pragma once

#include <QWidget>
#include <QPen>

class reticle :public QWidget
{
	
	Q_OBJECT
	
public :
	reticle(QWidget *parent = 0);
	void setWingsLevel(bool level);
	~reticle();
	
protected:
	void paintEvent(QPaintEvent *event) override;
	bool wingsLevel;
};


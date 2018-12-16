#pragma once

#include <QWidget>
#include <QPen>

#define NUM_SS_VALS  10

class slipskid_instrument :public QWidget
{
	
public:
	slipskid_instrument(QWidget *parent);
	~slipskid_instrument();
	
	void setValue(float val);
	
protected: 
	void paintEvent(QPaintEvent *event) override;
	
private:
	float vals[NUM_SS_VALS];
	int index;
	QPen pen;
	QColor penColor;
	float value; /* number of balls out */
	
};


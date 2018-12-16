#pragma once
#include <QWidget>

class StatusWidget:public QWidget
{
public:
	StatusWidget(QWidget *parent);
	~StatusWidget();
	QString status;
	bool show = false;
	int value = 0;
	
	void setValue(int val);
	
protected: 
	void paintEvent(QPaintEvent *event) override;
};


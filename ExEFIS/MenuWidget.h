#pragma once
#include <QWidget>

struct menuItem
{
	QString str;
	bool alwaysShow;	
};

class MenuWidget:public QWidget
{
public:
	MenuWidget(QWidget *parent);
	~MenuWidget();
	QString status;
	bool show = false;
	int value = 0;
	void setStatus(int arg1, const QString &arg2);
	void setValue(int val);
	
private:
	int index = 0; //the index for the menu item
	menuItem items[20];
	int numItems = 0;
	
protected: 
	void paintEvent(QPaintEvent *event) override;
	
	
};


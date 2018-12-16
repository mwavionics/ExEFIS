#pragma once

#include <QWidget>
#include <QTimer>
#include "adhrs.h"
#include "knobs.h"

class SplashWidget : public QWidget
{
	Q_OBJECT
		
public:
	SplashWidget(QWidget *parent = 0);
	SplashWidget(QWidget *parent, adhrs *ad,	knobs *k);
	~SplashWidget();
	
	public slots :
		void onTimerExp(void);
	
signals:
	void launchPanel(int index);
	
protected: 
	void paintEvent(QPaintEvent *event) override;
	void showEvent(QShowEvent *event) override;
	
private:
	QTimer* timer;
	adhrs *adhr;
	knobs *knob;
	
};


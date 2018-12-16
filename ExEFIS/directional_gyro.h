#pragma once
#include <QWidget.h>
#include <QTimer>
#include <QPen.h>
#include <QPainter>

class directional_gyro :public QWidget
{
	Q_OBJECT
		
public :
	directional_gyro(QWidget *parent = 0);
	~directional_gyro();
	int value;
	int setting;
	bool editMode = false;
	
	void toggleEditMode()
	{
		editMode = !editMode;
		if (editMode) blinkTimer->start();
		else
		{
			blinkTimer->stop();
			blinking = false;
		}
	}
	
	void setSetting(int arg1)
	{
		int tempsetting = arg1;
		while (tempsetting > 360) tempsetting -= 360;
		while (tempsetting < 0) tempsetting += 360;
		setting = tempsetting;
	}
	
	public slots :
			void onBlinkTimer(void)
	{
		blinking = !blinking;
		update();
	}
	
	
protected: 
	void paintEvent(QPaintEvent *event) override;
	
	
private:
	QColor penColor;
	QTimer *blinkTimer;
	bool blinking = false;
	
	float values[12] = 
	{ 
		3.0f,6.0f,9.0f,12.0f,15.0f,18.0f,21.0f,24.0f,27.0f,30.0f,33.0f,36.0f		
	};

};


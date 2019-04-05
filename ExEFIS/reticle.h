#pragma once

#include <QWidget>
#include <QPen>
#include <QTimer>

class reticle :public QWidget
{
	
	Q_OBJECT
	
public :
	reticle(QWidget *parent = 0);
	void setWingsLevel(bool level);
	~reticle();
	
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
		if (tempsetting > 20)tempsetting = 20;
		if (tempsetting < -20) tempsetting = -20;		
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
	bool wingsLevel;
	
private:
	QTimer *blinkTimer;
	bool blinking = false;
};


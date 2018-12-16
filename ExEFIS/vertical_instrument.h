#pragma once

#include <QWidget.h>
#include <QTimer>
#include <QPen.h>

class vertical_instrument :public QWidget
{
	Q_OBJECT
		
public :
	vertical_instrument(QWidget *parent = 0);
	vertical_instrument(QWidget *parent, QColor c);
	void setValue(int val);
	int getValue(void);
	void setupInstrument(int* vals, int numVals);
	void setSetting(float stg);
	void setEditMode(bool emode);
	void toggleEditMode(void);
	void change(void);
	~vertical_instrument();
	
	bool editMode = false;
	int setting = 0;
	int settingPrec = 2;
	bool showSetting = false;
	
public slots :
	void onBlinkTimer(void);
	
protected: 
	void paintEvent(QPaintEvent *event) override;
	
private:
	QPen pen;
	QColor penColor;
	QTimer *blinkTimer;
	bool blinking = false;
	
	int vertical_divs = 4;
	int value = 81;
	

	
	
	int numValues = 15;
	/* default values */
	int values[30] = { 
		0,
		10,
		20,
		30,
		40,
		50,
		60,
		70,
		80,
		90,
		100,
		110,
		120,
		130,
		140,
		150,
		160,
		170,
		180,
		190,
		200,
		210,
		220,
		230,
		240,
		250,
		260,
		270,
		280,
		290
	};
	
};


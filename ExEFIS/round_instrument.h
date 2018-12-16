#pragma once

#include <QWidget.h>
#include <QTimer>
#include <QPen.h>

class round_instrument : public QWidget
{
	Q_OBJECT
public:
	round_instrument(QWidget *parent = 0);
	round_instrument(QWidget *parent, QColor c);
	void setValue(int val);
	void setupInstrument(int* vals, int numVals);
	void setSetting(float stg);
	float getSetting(void);
	void setEditMode(bool emode);
	void toggleEditMode(void);
	void change(void);
	~round_instrument();
	
	bool editMode = false;
	int setting = 0;
	int settingPrec = 1;
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
	
	int number_divs = 12;
	int value = 180;
	
	int numValues = 12;
	
	int values[36] = { 
		3,
		6,
		9,
		12,
		15,
		18,
		21,
		24,
		27,
		30,
		33,
		36,
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
		290, 
		300,
		310,
		320,
		330,
		340,
		350,
	};
};


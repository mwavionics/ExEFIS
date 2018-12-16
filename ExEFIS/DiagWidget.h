#pragma once

#include <QWidget>
#include <QPushButton>
#include <QTextEdit>
#include "adhrs.h"
#include "knobs.h"


class DiagWidget : public QWidget
{
    Q_OBJECT
    
public:
    DiagWidget(QWidget *parent = 0);
    ~DiagWidget();
	void setADHRS(adhrs *a);
	void setKnobs(knobs *k);

protected slots:
    void ButtonClicked();
	void readRegisterButtonClicked();
	void readCalibrationButtonClicked();
	void pollTimerExp(void);
	
signals:
	void closeStacked(void);

protected:
	void showEvent(QShowEvent *event) override;
	
private:
	knobs *knob;
	adhrs *adhr;
	QPushButton *sendButton;
	QPushButton *readRegisterButton;
	QPushButton *readCalibrationButton;
	QTextEdit *textEditadhrs;
	QTextEdit *textEditknobs;
	QTextEdit *textEditconfig;
	QTimer *polltimer;
};


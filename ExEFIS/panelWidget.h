#pragma once
#include <QTimer>
#include <QWidget>
#include "horizon_instrument.h"
#include "vertical_instrument.h"
#include "reticle.h"
#include "slipskid_instrument.h"
#include "directional_gyro.h"
#include "round_instrument.h"
#include "StatusWidget.h"
#include "MenuWidget.h"
#include "adhrs.h"
#include "knobs.h"

class panelWidget : public QWidget
{
	Q_OBJECT
	
public:
	panelWidget(QWidget *parent = 0);
	~panelWidget();
	void setADHRS(adhrs* a);
	void setKNOBS(knobs* k);
	
public slots :
	void onTimer(void);
	void onDebugTimer(void);
	
signals:
	void launchDiag(int index);

protected:	
	void showEvent(QShowEvent *event) override;
	
private:
	QTimer *qtimer;
	QTimer *qDebugTimer;
	horizon_instrument *h1;
	round_instrument *r1;
	vertical_instrument *vi1;
	vertical_instrument *vi2;
	slipskid_instrument *ss;
	directional_gyro *dg;
	reticle *r;
	StatusWidget *sw;
	MenuWidget *mw;
	adhrs *adhr;
	knobs *knob;
};


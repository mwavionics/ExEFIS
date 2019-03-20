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

typedef struct
{
	int skyColor[4];
	int earthColor[4];
	int encoderConfig;
	int horizonOffset;
	int AltimeterSetting;
	int SteerCard[12];
}PANEL_SETTINGS;

class panelWidget : public QWidget
{
	Q_OBJECT
	
public:
	panelWidget(QWidget *parent = 0);
	panelWidget(QFile* f);
	~panelWidget();
	void setADHRS(adhrs* a);
	void setKNOBS(knobs* k);
	void setSettingsFile(QFile* file);
	
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
	QFile* settingsFile;
	PANEL_SETTINGS settings;
	
	
	
	bool loadSettingsFile();
	void settingsFile_ProcessLine(QByteArray line);	
	bool saveSettingsFile();
	bool defaultSettings();
};


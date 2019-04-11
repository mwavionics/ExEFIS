#include "panelWidget.h"
#include "horizon_instrument.h"
#include "vertical_instrument.h"
#include "directional_gyro.h"
#include "reticle.h"
#include "airspeed.h"
#include "altitude.h"
#include <QWidget>
#include <QGridLayout>
#include <QDebug>
#include <QString>
#include <QFile>
#include <math.h>


int counter = 0;
int mode = 0;

QGridLayout *mainLayout;

panelWidget::panelWidget(QWidget *parent, QFile* f, knobs *k)
	: QWidget(parent)
{
	setCursor(Qt::BlankCursor);		
	
	settingsFile = f;
	if (!loadSettingsFile())
	{
		saveSettingsFile();
	}		

	QColor e = QColor::fromRgb(settings.earthColor[0],
		settings.earthColor[1],
		settings.earthColor[2],
		settings.earthColor[3]);
	QColor s = QColor::fromRgb(settings.skyColor[0],
		settings.skyColor[1],
		settings.skyColor[2],
		settings.skyColor[3]);
	
	knob = k;
	knob->right->setConfig(settings.encoderConfig);
	//knob->left->getPress(true);
	knob->right->getPress(true);
	
	/* SSK Note that this order matters and affects draw order*/
	
	h1 = new horizon_instrument(this, e, s, settings.horizonOffset);
	//r1 = new round_instrument(this);
	vi1 = new vertical_instrument(this, Qt::black);
	vi1->showSecondary = false;
	vi1->showSecondaryValue = false;

	int altvals[10] = { 0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000 };	
	vi2 = new vertical_instrument(this, Qt::black);
	vi2->setupInstrument(altvals, 10);
	vi2->setValue(6350);
	vi2->setting = settings.altimeterSetting;
	vi2->showSetting = true;
	
	ss = new slipskid_instrument(this);
	dg = new directional_gyro(this);
	dg->value = 7;
	r = new reticle(this);
	r->setSetting(settings.horizonOffset);
	h1->setVerticalOffset(r->setting);
	
	sw = new StatusWidget(this);
	mw = new MenuWidget(this);

	
	mainLayout = new QGridLayout(this);
	mainLayout->setColumnStretch(0, 1);
	mainLayout->setColumnStretch(1, 3);
	mainLayout->setColumnStretch(2, 1);
	mainLayout->setRowStretch(0, 1);
	mainLayout->setRowStretch(1, 1);
	mainLayout->setRowStretch(2, 3);
	mainLayout->setRowStretch(3, 1);
	mainLayout->setRowStretch(4, 1);
	
	
	mainLayout->addWidget(h1, 0, 0, 5, 3, 0);
	
	/* SSK - note the magic here is the 4th arg "1" that is the column span */
	mainLayout->addWidget(vi1,0 , 0, 5, 1, 0);
	mainLayout->addWidget(vi2, 0, 2, 5, 1, 0);
	mainLayout->addWidget(r, 2, 1, 1, 1, 0);
	mainLayout->addWidget(ss, 3, 1, 1, 1, 0);
	mainLayout->addWidget(dg, 0, 1, 2, 1, 0);
	mainLayout->addWidget(mw, 4, 1, 1, 1, 0);
	
	qtimer = new QTimer(this);
	qtimer->setInterval(50);
	connect(qtimer, SIGNAL(timeout()), this, SLOT(onTimer()));
	
	qDebugTimer = new QTimer(this);
	qDebugTimer->setInterval(500);
	connect(qDebugTimer, SIGNAL(timeout()), this, SLOT(onDebugTimer()));
	

}

panelWidget::~panelWidget()
{
}

int oldSWMode = 0;
int lastAlt = 0;
int last = 0;
int vsi[10];
int vsiIndex = 0;

void panelWidget::onTimer(void)
{
	AHRS_DATA data;
	int status = adhr->getDataSet(&data);	
	
	h1->setAzimuth(data.pitch); 
	h1->setAngle(data.roll);
	dg->value = data.heading;
	vi1->setValue(data.airspeed);
	vi2->setValue(data.altitude);
	vi2->setSecondaryValue((int)data.vsi);
	ss->setValue(data.slip);	
	r->setWingsLevel(adhr->getAcState().wingslevel);
/*
	if (sw->value == 0) r1->setValue(0);
	else
	{
		r1->setValue(adhrdata[2]);
	}
	
	*/
	
	
	counter++;
	if (counter >= 5)
	{
		counter = 0;
	
		
		if (mw->value == 5)
		{
			AC_STATE s = adhr->getAcState();
			mw->setStatus(5,
				"WL: " + QString::number(s.wingslevel, 'd', 1) + " PL: " + QString::number(s.pitchlevel, 'd', 1) + " PS: " + QString::number(s.pitchstill, 'd', 1) + "\r" + "\n" +
			"Tur: " + QString::number(s.turning, 'd', 1) + " BC: " + QString::number(s.ballcentered, 'd', 1) + " OneG: " + QString::number(s.oneG, 'd', 1));      // " Pres: " + QString::number(adhrdata[0], 'g', 4);
		}
		if (mw->value == 4)
		{
			//this used to update VSI
			//mw->setStatus(4, "VSI:  " +  QString::number(vsival, 'f', 0));
			mw->setStatus(4, " DG: " + QString::number(data.heading, 'd', 1) + " VSI: " + QString::number(data.vsi, 'd', 1));
		}
		
	}	
	
	if (knob->right->getSinglePress())
	{
		mode++;
		if (mode == 1)
		{
			vi2->toggleEditMode();
			knob->right->setValue(vi2->setting);
		}
		if (mode == 2)
		{
			vi2->toggleEditMode();
			adhr->setAltimeterSetting(vi2->setting, vi2->settingPrec);
			settings.altimeterSetting = vi2->setting;
			saveSettingsFile();
			//Commit file here
		}
		if (mode == 3)
		{
			dg->toggleEditMode();
			knob->right->setValue(dg->setting);
		}
		if (mode == 4)
		{
			dg->toggleEditMode();
		}
		if (mode == 5)
		{
			r->toggleEditMode();
			r->setSetting(h1->verticalOffset);
			knob->right->setValue(r->setting);
			adhr->setAttitudeOffset(r->setting);
		}
		if (mode == 6)
		{
			r->toggleEditMode();
			h1->setVerticalOffset(r->setting);
			settings.horizonOffset = r->setting;
			adhr->setAttitudeOffset(r->setting);
			saveSettingsFile();
		}
		//Show the menu
		if (mode == 7)
		{
			mw->show = true;
			knob->right->setValue(sw->value);
			oldSWMode = mw->value;
		}
		//Select the menu
		if (mode == 8)
		{
			mw->show = false;			
			if (mw->value == 0) 
			{				
				system("gpio write 26 1");
			}
			if (mw->value == 1)
			{
				system("gpio write 26 0");
				system("i2cset -y 1 0x2f 0x00 0x0000 w");
			}
			if (mw->value == 2)
			{				
				system("gpio write 26 0"); // HW Rev 2
				system("i2cset -y 1 0x2f 0x00 0xFFFF w");
			}	
			if (mw->value == 6)
			{
				exit(0);
			}
		}
		if (mode >= 8)
		{			
			mode = 0;	
		}
	}
/*	
	if (r1->editMode)
	{
		r1->setSetting(knob->right->getValue());  //NOTE FIXED FOR 3.5 Display
		knob->right->setValue(r1->getSetting());
	}
	*/
	if (vi2->editMode)
	{
		int rightknobvalue = knob->right->getValue();
		adhr->setAltimeterSetting(rightknobvalue, vi2->settingPrec);
		vi2->setSetting(rightknobvalue);		
	}
	if (mw->show)
	{
		mw->setValue(knob->right->getValue());
		knob->right->setValue(mw->value);
	}
	
	if (dg->editMode)
	{
		dg->setSetting(knob->right->getValue());   //NOTE FIXED FOR 3.5 Display
		knob->right->setValue(dg->setting);
	}
	
	if (r->editMode)
	{
		r->setSetting(knob->right->getValue());    //NOTE FIXED FOR 3.5 Display
		h1->setVerticalOffset(r->setting);
		knob->right->setValue(r->setting);
	}
	
	if (knob->right->getPress(false) > 10)
	{
		//qtimer->stop();
		//close();
		//launchDiag(2);		
	}
}


void panelWidget::setADHRS(adhrs* a)
{
	adhr = a;
	//Init the adhrs when we're ready, if you do this before, you wind up with a
	//strange almost "multithreaded" application for a bit...
	adhr->setAltimeterSetting(settings.altimeterSetting, vi2->settingPrec);
	adhr->setSteerToSettings(settings.steerCard);
	adhr->setAttitudeOffset(r->setting);
	adhr->Init();
}




void panelWidget::setSettingsFile(QFile* file)
{
	settingsFile = file;
}


void panelWidget::showEvent(QShowEvent *event)
{
	qtimer->start();
	//qDebugTimer->start();
}


void panelWidget::onDebugTimer(void)
{
	unsigned char offsets[22];
	char caldata[4];
	AHRS_DATA data;
//	int status = adhr->getDataSet(&data);
	
	printf("!!! Orientation !!!");
	printf("Pitch %3.2f", data.pitch);
	printf("Roll %3.2f" , data.roll);
	printf("Heading %3.2f" , data.heading);
	printf("Airspeed %3.2f", data.airspeed);
	printf("Altitude %5.0f" , data.altitude);
	
}


bool panelWidget::loadSettingsFile()
{
	bool success = false;
	defaultSettings();
	if (settingsFile != NULL)
	{	
		if (settingsFile->exists())
		{
			qDebug() << "Panel Settings file exists - processing..." << settingsFile->fileName();
			if (settingsFile->open(QIODevice::ReadOnly))
			{
				while (!settingsFile->atEnd()) {
					QByteArray line = settingsFile->readLine();
					settingsFile_ProcessLine(line);
				}
				/* do we calibrate?*/
				qDebug() << "valid settings File loaded";				
				success = true;
				
				settingsFile->close();
			}			
		}
	}
	if (!success) qDebug() << "***NO SETTINGS FILE FOUND***";
	return success;
}

bool panelWidget::saveSettingsFile()
{
	bool success = false;
	if (settingsFile != NULL)
	{	
		// no need to check if the file exists, open will create it
		settingsFile->open(QIODevice::ReadWrite);	
		
		settingsFile->write(QString().sprintf("Sky Color , %d, %d, %d, %d,\r\n", settings.skyColor[0], settings.skyColor[1], settings.skyColor[2], settings.skyColor[3]).toUtf8());
		settingsFile->write(QString().sprintf("Earth Color , %d, %d, %d, %d,\r\n", settings.earthColor[0], settings.earthColor[1], settings.earthColor[2], settings.earthColor[3]).toUtf8());
		settingsFile->write(QString().sprintf("Encoder Config , %d,\r\n", settings.encoderConfig ).toUtf8());
		settingsFile->write(QString().sprintf("Horizon Offset , %d,\r\n", settings.horizonOffset ).toUtf8());
		settingsFile->write(QString().sprintf("Altimeter Setting , %d,\r\n", settings.altimeterSetting ).toUtf8());
		settingsFile->write(QString().sprintf("For 030 Steer , %d,\r\n", settings.steerCard[0]).toUtf8()) ;
		settingsFile->write(QString().sprintf("For 060 Steer , %d,\r\n", settings.steerCard[1]).toUtf8());
		settingsFile->write(QString().sprintf("For 090 Steer , %d,\r\n", settings.steerCard[2]).toUtf8());
		settingsFile->write(QString().sprintf("For 120 Steer , %d,\r\n", settings.steerCard[3]).toUtf8());
		settingsFile->write(QString().sprintf("For 150 Steer , %d,\r\n", settings.steerCard[4]).toUtf8());
		settingsFile->write(QString().sprintf("For 180 Steer , %d,\r\n", settings.steerCard[5]).toUtf8());
		settingsFile->write(QString().sprintf("For 210 Steer , %d,\r\n", settings.steerCard[6]).toUtf8());
		settingsFile->write(QString().sprintf("For 240 Steer , %d,\r\n", settings.steerCard[7]).toUtf8());
		settingsFile->write(QString().sprintf("For 270 Steer , %d,\r\n", settings.steerCard[8]).toUtf8());
		settingsFile->write(QString().sprintf("For 300 Steer , %d,\r\n", settings.steerCard[9]).toUtf8());
		settingsFile->write(QString().sprintf("For 330 Steer , %d,\r\n", settings.steerCard[10]).toUtf8());
		settingsFile->write(QString().sprintf("For 360 Steer , %d,\r\n", settings.steerCard[11]).toUtf8());
		settingsFile->close();		
	}
	success = true;
}


void panelWidget::settingsFile_ProcessLine(QByteArray line)
{
	if (line.startsWith("Sky Color"))
	{		
		settings.skyColor[0] = line.split(',')[1].toInt();
		settings.skyColor[1] = line.split(',')[2].toInt();
		settings.skyColor[2] = line.split(',')[3].toInt();
		settings.skyColor[3] = line.split(',')[4].toInt();		
	}
	if (line.startsWith("Earth Color"))
	{		
		settings.earthColor[0] = line.split(',')[1].toInt();
		settings.earthColor[1] = line.split(',')[2].toInt();
		settings.earthColor[2] = line.split(',')[3].toInt();
		settings.earthColor[3] = line.split(',')[4].toInt();		
	}
	if (line.startsWith("Encoder Config")) settings.encoderConfig = line.split(',')[1].toInt();
	if (line.startsWith("Horizon Offset")) settings.horizonOffset = line.split(',')[1].toInt();
	if (line.startsWith("Altimeter Setting")) settings.altimeterSetting = line.split(',')[1].toInt();
	if (line.startsWith("For 030 Steer")) settings.steerCard[0] = line.split(',')[1].toInt();
	if (line.startsWith("For 060 Steer")) settings.steerCard[1] = line.split(',')[1].toInt();
	if (line.startsWith("For 090 Steer")) settings.steerCard[2] = line.split(',')[1].toInt();
	if (line.startsWith("For 120 Steer")) settings.steerCard[3] = line.split(',')[1].toInt();
	if (line.startsWith("For 150 Steer")) settings.steerCard[4] = line.split(',')[1].toInt();
	if (line.startsWith("For 180 Steer")) settings.steerCard[5] = line.split(',')[1].toInt();
	if (line.startsWith("For 210 Steer")) settings.steerCard[6] = line.split(',')[1].toInt();
	if (line.startsWith("For 240 Steer")) settings.steerCard[7] = line.split(',')[1].toInt();
	if (line.startsWith("For 270 Steer")) settings.steerCard[8] = line.split(',')[1].toInt();
	if (line.startsWith("For 300 Steer")) settings.steerCard[9] = line.split(',')[1].toInt();
	if (line.startsWith("For 330 Steer")) settings.steerCard[10] = line.split(',')[1].toInt();
	if (line.startsWith("For 360 Steer")) settings.steerCard[11] = line.split(',')[1].toInt();
	
}


bool panelWidget::defaultSettings()
{
	settings.skyColor[0] = 125;
	settings.skyColor[1] = 206;
	settings.skyColor[2] = 250;
	settings.skyColor[3] = 255; 
	settings.earthColor[0] = 114;
	settings.earthColor[1] = 59;
	settings.earthColor[2] = 34;
	settings.earthColor[3] = 255; 
	settings.encoderConfig = 1;
	settings.horizonOffset = 0;
	settings.altimeterSetting = 2992;
	settings.steerCard[0] = 30;
	settings.steerCard[1] = 60;
	settings.steerCard[2] = 90;
	settings.steerCard[3] = 120;
	settings.steerCard[4] = 150;
	settings.steerCard[5] = 180;
	settings.steerCard[6] = 210;
	settings.steerCard[7] = 240;
	settings.steerCard[8] = 270;
	settings.steerCard[9] = 300;
	settings.steerCard[10] = 330;
	settings.steerCard[11] = 360;
		
}


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
#include <math.h>


int counter = 0;
int mode = 0;

QGridLayout *mainLayout;

panelWidget::panelWidget(QWidget *parent)
	: QWidget(parent)
{
	setCursor(Qt::BlankCursor);	
	
	/* SSK Note that this order matters and affects draw order*/
	h1 = new horizon_instrument(this);
	//r1 = new round_instrument(this);
	vi1 = new vertical_instrument(this, Qt::black);

	int altvals[10] = { 0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000 };	
	vi2 = new vertical_instrument(this, Qt::black);
	vi2->setupInstrument(altvals, 10);
	vi2->setValue(6350);
	vi2->setting = 2992;
	vi2->showSetting = true;
	
	ss = new slipskid_instrument(this);
	dg = new directional_gyro(this);
	dg->value = 7;
	r = new reticle(this);
	
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
	vi1->setValue(airspeed::getIASMph(data.aspPressurePSI));
	vi2->setValue(altitude::getAltitudeFt(data.staticPressurePSI, vi2->setting/pow(10, vi2->settingPrec)));
	ss->setValue(data.slip);	
	r->setWingsLevel(adhr->getWingsLevel());
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
			mw->setStatus(5,
			"Pitch: " + QString::number(data.pitch, 'f', 2) + " Roll: " + QString::number(data.roll, 'f', 2) + "\r" + "\n" +
			"SS: " + QString::number(data.slip, 'f', 2) + " Hdg: " + QString::number(data.heading, 'f', 2));   // " Pres: " + QString::number(adhrdata[0], 'g', 4);
		}
		if (mw->value == 4)
		{
			//this used to update VSI
			//mw->setStatus(4, "VSI:  " +  QString::number(vsival, 'f', 0));
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
		//Show the menu
		if (mode == 5)
		{
			mw->show = true;
			knob->right->setValue(sw->value);
			oldSWMode = mw->value;
		}
		//Select the menu
		if (mode == 6)
		{
			mw->show = false;			
			if (mw->value == 0) 
			{				
				system("gpio write 26 1");
			}
			if (mw->value == 2)
			{
				system("gpio write 26 0");
				system("i2cset -y 1 0x2f 0x00 0x0000 w");
			}
			if (mw->value == 1)
			{				
				system("gpio write 26 0"); // HW Rev 2
				system("i2cset -y 1 0x2f 0x00 0xFFFF w");
			}	
			if (mw->value == 6)
			{
				exit(0);
			}
		}
		if (mode >= 6)
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
	if (vi2->editMode)vi2->setSetting(knob->right->getValue());
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
}


void panelWidget::setKNOBS(knobs* k)
{
	/* be sure and read the presses and clear them when you set the variable*/
	knob = k;
	//knob->left->getPress(true);
	knob->right->getPress(true);
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
	int status = adhr->getDataSet(&data);
	
	printf("!!! Orientation !!!");
	printf("Pitch %3.2f", data.pitch);
	printf("Roll %3.2f" , data.roll);
	printf("Heading %3.2f" , data.heading);
	printf("Airspeed %3.2f", airspeed::getIASMph(data.aspPressurePSI));
	printf("Altitude %5.0f" , altitude::getAltitudeFt(data.staticPressurePSI, vi2->setting / pow(10, vi2->settingPrec)));
	
}

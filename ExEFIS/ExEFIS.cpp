#include <QApplication>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QProcess>
#include <QWidget>
#include <QScreen>
#include <QThread>
#include <QDebug>
#include <iostream>
#include <pigpio.h>

#include "SplashWidget.h"
#include "panelWidget.h"
#include "DiagWidget.h"
#include "knobs.h"
#include "adhrs.h"

	
int main(int argc, char *argv[])
{	
	QStringList list = QProcessEnvironment::systemEnvironment().toStringList();		
	
	/* Start some String outputs in case you are launching the app with loggint enabled */
	qDebug("****************** LOG BEGINS HERE **************************");
	qDebug("Version 5.2 - Continued Test - fixed ss with remap");
	
	system("cat /proc/cpuinfo | grep Serial | cut -d ' ' -f 2");
	system("gpio mode 26 out");
	system("gpio write 26 0");
	system("i2cset -y 1 0x2f 0x00 0x0000 w");
	
	QApplication a(argc, argv);
	
	QApplication::setOverrideCursor(Qt::BlankCursor);
	
	QScreen *screen = QGuiApplication::primaryScreen();
	QRect  screenGeometry = screen->geometry();
	
	/* Now that we're using pigpio, initialize the library */
	if (gpioInitialise() < 0) qDebug("ERROR: Can't initialize");
	
	knobs *k = new knobs();	
	adhrs *ad = new adhrs();	
	
	QStackedWidget *w = new QStackedWidget();
	
	panelWidget *p = new panelWidget();
	p->setADHRS(ad);
	p->setKNOBS(k);
	//p->showFullScreen();
	//p->update();
	
	p->setCursor(Qt::BlankCursor);
	
	
	SplashWidget *s = new SplashWidget(0, ad, k);

	
	//m->setADHRS(ad);
	//m->setKnobs(k);
	//m->showFullScreen();
	
	w->addWidget(s);
	w->addWidget(p);
//	w->addWidget(m);
	

	//QStackedWidget::connect(p, SIGNAL(launchDiag(int)), w, SLOT(setCurrentIndex(int)));
	QStackedWidget::connect(s, SIGNAL(launchPanel(int)), w, SLOT(setCurrentIndex(int)));
	w->setCurrentIndex(0);

	w->showFullScreen();
	

	
    return a.exec();
}

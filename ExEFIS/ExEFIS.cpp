#include <QApplication>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QProcess>
#include <QWidget>
#include <QString>
#include <QScreen>
#include <QThread>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <iostream>
#include <pigpio.h>
#include <unistd.h>

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
	qDebug("Version 5.4 - Added settings file and Attitude Adjust - added attitude offset for Filter ");
	
	system("cat /proc/cpuinfo | grep Serial | cut -d ' ' -f 2");
	system("gpio mode 26 out");
	system("gpio write 26 0");
	system("i2cset -y 1 0x2f 0x00 0x0000 w");
	
	QApplication a(argc, argv);
	
	QApplication::setOverrideCursor(Qt::BlankCursor);
	
	QScreen *screen = QGuiApplication::primaryScreen();
	QRect  screenGeometry = screen->geometry();
	
	qDebug("numoptions is %d", argc);
	for (int i = 0; i < argc; i++)
	{
		qDebug() << argv[i];
	}
	
	int opt;
	bool domagcal = false;
	bool showmagvector = false;
	while ((opt = getopt(argc, argv, "hmgac:")) != -1)  
	{  
		switch (opt)  
		{  
		case 'm': 
			domagcal = true;
			qDebug("Option m passed - will do magcal test");
			break;	
		case 'g': 
			showmagvector = true;
			qDebug("Option g passed - will show mag vectors");
			break;	
		case 'h':
			qDebug() << "-m for magnetometer calibration";
			qDebug() << "-g for magnetometer vector verification";
			qDebug() << "-a ... undefined";
			qDebug() << "-c ... undefined";
				
		}  
	}  
	
	/* Now that we're using pigpio, initialize the library */
	if (gpioInitialise() < 0) qDebug("ERROR: Can't initialize");
	
	
	/* Search the /home/pi directory for a sensor cal file
	 * This has been udpated to allow sensorcal_serialnumber.txt files
	 * This way MW Avionics can keep sensorcal files as backups for each SN*/
	bool cal = false;
	AHRS_CAL* pCal = NULL;
	QFile* calfile;
	QDir directory("/home/pi");
	qDebug() << "Searching " << directory.path() << " for calibration file";
	QFileInfoList files = directory.entryInfoList(QStringList() << "*.txt" << "*.TXT", QDir::Files);
	QString filename = NULL;
	int num = files.count();
	for (int i = 0; i < num; i++)
	{
		QString fn = files.at(i).fileName();		
		if (fn.contains("sensorcal", Qt::CaseInsensitive))
		{
			filename = files.at(i).filePath();			
		}
	}
	if (filename != NULL)
	{
		calfile = new QFile(filename);		
	}
	
	QFile* settingsFile;	
	qDebug() << "Searching " << directory.path() << " for settings file";
	files = directory.entryInfoList(QStringList() << "*.txt" << "*.TXT", QDir::Files);	
	num = files.count();
	QString fname = NULL;
	for (int i = 0; i < num; i++)
	{
		QString fn = files.at(i).fileName();		
		if (fn.contains("settings", Qt::CaseInsensitive))
		{
			fname = files.at(i).filePath();			
		}
	}
	if (fname != NULL)
	{
		settingsFile = new QFile(fname);		
	}
	else
	{
		settingsFile = new QFile("/home/pi/settingsfile.txt");		
	}
	
	knobs *k = new knobs();	
	pCal = adhrs::processCalibrationFile(calfile);
	adhrs *ad = new adhrs(pCal, domagcal, showmagvector);	
	
	QStackedWidget *w = new QStackedWidget();
	
	panelWidget *p = new panelWidget(NULL, settingsFile, k);
	p->setADHRS(ad);

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

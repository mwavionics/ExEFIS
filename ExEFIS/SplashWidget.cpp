#include "SplashWidget.h"
#include "panelWidget.h"
#include "DiagWidget.h"
#include "adhrs.h"
#include "knobs.h"
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QMessageBox>

bool launchDebug = false;

SplashWidget::SplashWidget(QWidget *parent)
	:QWidget(parent)
{
	
	timer = new QTimer(this);
	
	connect(timer, SIGNAL(timeout()), this, SLOT(onTimerExp()));
	timer->start(5000);
	update();
}


SplashWidget::SplashWidget(QWidget *parent, 
	adhrs *ad, 
	knobs *k)
	:QWidget(parent)
{
	adhr = ad;
	knob = k;
	k->left->getPress(true);
	
	timer = new QTimer(this);
	
	connect(timer, SIGNAL(timeout()), this, SLOT(onTimerExp()));	
}


SplashWidget::~SplashWidget()
{
}

void SplashWidget::paintEvent(QPaintEvent *e)
{
	QPixmap pixmap2(":/img/logo.png");
	QPainter painter(this);
	painter.drawPixmap(0, 0, width(), height(), pixmap2);
}


void SplashWidget::onTimerExp(void)
{
	timer->stop();
	
	
	if (knob->left->getPress(true) >= 3) launchDebug = true;
	if (launchDebug)
	{
		QMessageBox msgBox;
		msgBox.setText("Launching Diagnostics...");
		msgBox.setWindowTitle("");
		msgBox.exec();
		this->close();
		launchPanel(2);
	}	
	else
	{
		this->close();
		launchPanel(1);
	}
	
}




void SplashWidget::showEvent(QShowEvent *event)
{
	timer->start(3000);
	update();
}

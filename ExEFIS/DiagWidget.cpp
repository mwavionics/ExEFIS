#include "DiagWidget.h"
#include <QMessageBox>
#include <QGridLayout>
#include <QWidget>
#include <QTimer>
#include <QApplication>

DiagWidget::DiagWidget(QWidget *parent)
   : QWidget(parent)
{
	/* SSK - this is CRUCIAL!!! Can't create a Layout on a Window - only on the central widget*/
	//setCentralWidget(new QWidget(this));
	
	sendButton = new QPushButton(this);
	sendButton->setText("Quit");
	sendButton->setFixedHeight(20);
	connect(sendButton, SIGNAL(pressed()), this, SLOT(ButtonClicked()));
	
	readRegisterButton = new QPushButton(this);
	readRegisterButton->setText("Read Reg");
	readRegisterButton->setFixedHeight(20);
	connect(readRegisterButton, SIGNAL(pressed()), this, SLOT(readRegisterButtonClicked()));
	
	readCalibrationButton = new QPushButton(this);
	readCalibrationButton->setText("Read Cal");
	readCalibrationButton->setFixedHeight(20);
	connect(readCalibrationButton, SIGNAL(pressed()), this, SLOT(readCalibrationButtonClicked()));
	
	textEditadhrs = new QTextEdit(this);
	textEditadhrs->setText("ADHRS Data");
	
	textEditknobs = new QTextEdit(this);
	textEditknobs->setText("Knobs Data");
	
	textEditconfig = new QTextEdit(this);
	textEditconfig->setText("Config Settings");
	
	QGridLayout *mainLayout = new QGridLayout;
	mainLayout->setColumnStretch(0, 1);
	//mainLayout->setColumnStretch(1, 1);
	//mainLayout->setColumnStretch(2, 2);
	//mainLayout->setColumnStretch(3, 1);
	mainLayout->addWidget(textEditknobs, 0, 0, 3, 1, 0);
//	mainLayout->addWidget(textEditknobs, 0, 1, 3, 1, 0);
	//mainLayout->addWidget(textEditconfig, 0, 2, 3, 1, 0);
//	mainLayout->addWidget(sendButton, 0, 3, 1, 1, 0);
//	mainLayout->addWidget(readRegisterButton, 1, 3, 1, 1, 0);
//	mainLayout->addWidget(readCalibrationButton, 2, 3, 1, 1, 0);
	
	setLayout(mainLayout);

	polltimer = new QTimer(this);
	polltimer->setInterval(200);
	connect(polltimer, SIGNAL(timeout()), this, SLOT(pollTimerExp()));
	
}

DiagWidget::~DiagWidget()
{
   
}

void DiagWidget::ButtonClicked()
{
    QMessageBox msgBox;
    msgBox.setText("Shutting Down");
    msgBox.setWindowTitle("WARNING!!!");
    msgBox.exec();
	QApplication::exit();
}

void DiagWidget::setADHRS(adhrs *a)
{
	adhr = a;
}


void DiagWidget::setKnobs(knobs *k)
{
	knob = k;
}

void DiagWidget::pollTimerExp(void)
{
	
	adhr->readAll();
	float adhrdata[6];
	int status = adhr->getAllSixRaw(adhrdata);
	
	textEditadhrs->setText(
		QString("Static Press (psi)") + '\r' + '\n' +
		QString::number(adhrdata[0], 'f', 3) + '\r' + '\n' +
		"Pitot Press (mBar)" + '\r' + '\n' +
		QString::number(adhrdata[1], 'f', 4) + '\r' + '\n' +
		+ '\r' + '\n' +
		"Orientation (deg)" + '\r' + '\n' +
		QString::number(adhrdata[2], 'f', 3) + '\r' + '\n' +
		QString::number(adhrdata[3], 'f', 3) + '\r' + '\n' + 
		QString::number(adhrdata[4], 'f', 3) + '\r' + '\n'  +
		+ '\r' + '\n'  +
		"Slip Skid in X" + '\r' + '\n' +
		QString::number(adhrdata[5], 'f', 2)
		);
	
	textEditknobs->setText(QString::number( knob->right->getValue() ) + '\r' + '\n' +
		QString::number(knob->right->getPress(false)) + '\r' + '\n' +
		QString::number(knob->right->getValue() ) + '\r' + '\n' +
		QString::number(knob->right->getPress(false)));
	//textEdit2->setText(QString::number(pressure));
	
}

void DiagWidget::showEvent(QShowEvent *event)
{
	polltimer->start();
}


void DiagWidget::readRegisterButtonClicked()
{
	bool hexstatus = false;
	uint reg = this->textEditconfig->toPlainText().toUInt(&hexstatus, 16); 
	if (hexstatus)
	{
		uint value = 0; //= adhr->readBNORegister(reg);
		this->textEditconfig->setText(textEditconfig->toPlainText() + '\r' + '\n' + QString::number(value, 16));
		
	}
}


void DiagWidget::readCalibrationButtonClicked()
{
	char calData[22];
	if (adhr->getOffsets(calData) == 1)
	{
		this->textEditconfig->setText("");
		for (int i = 0; i < 22; i++)
		{
			this->textEditconfig->append(QString::number(calData[i], 16));
		}
	}
}

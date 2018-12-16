#include "adhrs.h"
#include "HRS_9250.h"
#include "hsc_pressure.h"

#include <QApplication>
#include <QFile>
#include <QByteArray>
#include <QString>
#include <math.h>
#include <QDebug>
#include <QTimer>
#include <QWidget>
#include "wiringPi.h"
#include <stdio.h>


adhrs::adhrs()
{
	
	staticpress = new hsc_pressure(0);
	staticpress->set_params(15, 0);
	airspeed = new hsc_pressure(1);
	airspeed->set_params(1, -1);

	/* Current Kitfox Values 10/12/18 */
	static float gyroBias[3];
	gyroBias[0] = 0.70;
	gyroBias[1] = 0.68;
	gyroBias[2] = 0.60;	
	
	static float accelBias[3];
	accelBias[0]  = -0.021;
	accelBias[1] = 0.026;
	accelBias[2] = -0.24;
	
	static float magBias[3];
	magBias[0] = 79.596497;
	magBias[1] = 213.663010;
	magBias[2] = -576.351318;
	
	static float magScale[3];
	magScale[0] = 0.984127;
	magScale[1] = 0.988048;
	magScale[2] = 1.029046;
	
/* ****END Current Kitfox values 10/12/18**** */
	
	
/* Current Test Unit Values     */
//	static float gyroBias[3];
//	gyroBias[0] = -1.8;
//	gyroBias[1] = 0.25;
//	gyroBias[2] = .55;
//	
//	static float accelBias[3];
//	accelBias[0]  = -0.019;
//	accelBias[1] = 0.023;
//	accelBias[2] = 0.050;
//	
//	static float magBias[3];
//	magBias[0] = -68.9835;
//	magBias[1] = 236.0305;
//	magBias[2] = -290.741;
//	
//	static float magScale[3];
//	magScale[0] = 0.981525;
//	magScale[1] = 1.053538;
//	magScale[2] = 0.968996;
	
/* ****END Current Test Unit values 10/12/18**** */
	
//	X - Axis sensitivity adjustment value + 1.18
//Y - Axis sensitivity adjustment value + 1.19
//Z - Axis sensitivity adjustment value + 1.14
		
	bool cal = false;
	QFile *calfile = new QFile("/home/pi/sensorcal.txt");
	if (calfile->exists())
	{
		if (calfile->open(QIODevice::ReadOnly))
		{
			while (!calfile->atEnd()) {
				QByteArray line = calfile->readLine();
				calfile_process_line(line, caldata);
			}
			/* do we calibrate?*/
			cal = calfile_validate(caldata);			
		}
	}
	
	if (cal) 
	{
		qDebug() << "found valid caldata" << calfile->fileName();		
		HRS_9250 *hrs = new HRS_9250(&caldata[0], &caldata[3], &caldata[6], &caldata[9]);		
		int s = hrs->Init(true, false, false);
	}
	else
	{		
		HRS_9250 *hrs = new HRS_9250(gyroBias, accelBias, magBias, magScale);	
		int s = hrs->Init(true, false, false);
	}		
}


adhrs::~adhrs()
{
}


void adhrs::readAll(void)
{	
	int error;
	this->euHeading = hrs->getHeading(); 		
	this->euRoll = hrs->getRoll();
	this->euPitch = hrs->getPitch();
	staticPressurePSI = staticpress->getPressure();
	aspPressureMBAR = airspeed->getPressure();
	imu::Vector<3> a = hrs->GetAccelerometer(&error);
	slipRAW = -8.0f*(a.y());	
	
	
	if (std::isnan(this->euHeading))
	{
		hrs->resetAlgorithm();
	}
}


int adhrs::getAllSixRaw(float* data)
{		
	
	int status = 0;
	data[0] = this->staticPressurePSI;
	data[1] = this->aspPressureMBAR;
	data[2] = (this->euHeading * (180.0f / M_PI)); //quat is in radians
	data[3] = (this->euRoll * (180.0f / M_PI)); //quat is in radians
	data[4] = (this->euPitch * (180.0f / M_PI)); //quat is in radians
	data[5] = (this->slipRAW);
	
	return status;
}

int adhrs::getOffsets(char* calData)
{
	if (true) //get offsets)
	{
		
		return 1;
	}
	return 0;
}


int adhrs::setOffsets(char* calData)
{
	//set offsets
	return 1;
}


void adhrs::getCalibration(char* cal)
{	
	//get calibration
}


void adhrs::calfile_process_line(QByteArray &line, float* data)
{
	if (line.startsWith("Gyro Offset X"))
	{		
		float val = line.split('"')[1].toFloat();
		data[0] = val;
	}
	if (line.startsWith("Gyro Offset Y"))
	{		
		float val = line.split('"')[1].toFloat();
		data[1] = val;
	}
	if (line.startsWith("Gyro Offset Z"))
	{		
		float val = line.split('"')[1].toFloat();
		data[2] = val;
	}
	if (line.startsWith("Accel Offset X"))
	{		
		float val = line.split('"')[1].toFloat();
		data[3] = val;
	}
	if (line.startsWith("Accel Offset Y"))
	{		
		float val = line.split('"')[1].toFloat();
		data[4] = val;
	}
	if (line.startsWith("Accel Offset Z"))
	{		
		float val = line.split('"')[1].toFloat();
		data[5] = val;
	}
	
	if (line.startsWith("Mag Offset X"))
	{		
		float val = line.split('"')[1].toFloat();
		data[6] = val;
	}
	if (line.startsWith("Mag Offset Y"))
	{		
		float val = line.split('"')[1].toFloat();
		data[7] = val;
	}
	if (line.startsWith("Mag Offset Z"))
	{		
		float val = line.split('"')[1].toFloat();
		data[8] = val;
	}
	if (line.startsWith("Mag Scale X"))
	{		
		float val = line.split('"')[1].toFloat();
		data[9] = val;
	}
	if (line.startsWith("Mag Scale Y"))
	{		
		float val = line.split('"')[1].toFloat();
		data[10] = val;
	}
	if (line.startsWith("Mag Scale Z"))
	{		
		float val = line.split('"')[1].toFloat();
		data[11] = val;	
	}
}


bool adhrs::calfile_validate(float* data)
{
	return (true);
}


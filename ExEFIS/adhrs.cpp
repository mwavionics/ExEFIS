#include "adhrs.h"
#include "mpudriver.h"
#include "hsc_pressure.h"
#include <QApplication>
#include <QFile>
#include <QDir>
#include <QByteArray>
#include <QString>
#include <math.h>
#include <QDebug>
#include <QTimer>
#include <QWidget>


/**********************************************************************************************//**
* Module: adhrs::adhrs
***************************************************************************************************
* @brief	Constructor for the adhrs class
* @note		
* @todo		
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 11/06/17 - Created & Commented
**************************************************************************************************/
adhrs::adhrs()
{	
	staticpress = new hsc_pressure();
	staticpress->set_params(15, 0);
	airspeed = new hsc_pressure(1);
	airspeed->set_params(1, -1);

	//NOTE: these are static so that the pigpio callback for the interrupt has access to them
	static float gyroBias[3];
	gyroBias[0] = 0.0;
	gyroBias[1] = 0.0;
	gyroBias[2] = 0.0;	
	
	static float accelBias[3];
	accelBias[0] = 0;
	accelBias[1] = 0;
	accelBias[2] = 0;
	
	static float magBias[3];
	magBias[0] = 0;
	magBias[1] = 0;
	magBias[2] = 0;
	
	static float magScale[3];
	magScale[0] = 1;
	magScale[1] = 1;
	magScale[2] = 1;	
	
	//Initialize the caldata to all invalid values
	for (int i = 0; i < 12; i++)
	{
		caldata[i] = 10000.0f;
	}	
	
	/* Search the /home/pi directory for a sensor cal file
	 * This has been udpated to allow sensorcal_serialnumber.txt files
	 * This way MW Avionics can keep sensorcal files as backups for each SN*/
	bool cal = false;
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
		QFile *calfile = new QFile(filename);
		if (calfile->exists())
		{
			qDebug() << "calfile exists" << calfile->fileName();
			if (calfile->open(QIODevice::ReadOnly))
			{
				while (!calfile->atEnd()) {
					QByteArray line = calfile->readLine();
					calfile_process_line(line, caldata);
				}
				/* do we calibrate?*/
				cal = calfile_validate(caldata);
				if (cal) 
				{
					qDebug() << "found valid caldata" << calfile->fileName();
					mpudriver *hrs = new mpudriver(&caldata[0], &caldata[3], &caldata[6], &caldata[9]);				
					int s = hrs->Init(true, false, false);
				}				
			}			
		}		
	}
	if (!cal)
	{			
		qDebug() << "using zero'd caldata, no file found" ;
		mpudriver *hrs = new mpudriver(gyroBias, accelBias, magBias, magScale);				
		int s = hrs->Init(true, false, false);		
	}
}

/**********************************************************************************************//**
* Module: adhrs::~adhrs
***************************************************************************************************
* @brief	DeConstructor for the adhrs class
* @note		
* @todo		
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 11/06/17 - Created & Commented
**************************************************************************************************/
adhrs::~adhrs()
{
}

/**********************************************************************************************//**
* Module: adhrs::getAllSixRaw
***************************************************************************************************
* @brief	Get all 6 elements needed for the panel
* @note		
* @todo		
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 11/06/17 - Created & Commented
**************************************************************************************************/
int adhrs::getDataSet(AHRS_DATA* data)
{			
	int status =1;
	int error = 1;
	int retry = 0;
	while (error && retry < 3)
	{		
		error = 0;
		if (!error)
		{	
			data->staticPressurePSI = staticpress->getPressure();
			data->aspPressurePSI = airspeed->getPressure();
			data->heading = hrs->getHeading() * (180.0f / M_PI);
			data->roll = hrs->getRoll() * (180.0f / M_PI);
			data->pitch = hrs->getPitch() * (180.0f / M_PI);
			float ay = hrs->GetYAccelFiltered(&error);
			data->slip = -8.0f*(ay);		
		
		}
		retry++;
	}
	if (error || retry >=3)
	{		
		status = 0;
		qDebug() << "Read Error - 3 retrys failed" << QString::number(error, 10) << ","; 
	}
	
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
	bool valid = true;
	//Gyro
	if (data[0] > GYRO_POS_VALID || data[0] < GYRO_NEG_VALID) valid = false;
	if (data[1] > GYRO_POS_VALID || data[1] < GYRO_NEG_VALID) valid = false;
	if (data[2] > GYRO_POS_VALID || data[2] < GYRO_NEG_VALID) valid = false;
	
	//Accel	
	if (data[3] > ACCEL_POS_VALID || data[3] < ACCEL_NEG_VALID) valid = false;
	if (data[4] > ACCEL_POS_VALID || data[4] < ACCEL_NEG_VALID) valid = false;
	if (data[5] > ACCEL_POS_VALID || data[5] < ACCEL_NEG_VALID) valid = false;
	
	//Mag Offset
	if (data[6] > MAGBIAS_POS_VALID || data[6] < MAGBIAS_NEG_VALID) valid = false;
	if (data[7] > MAGBIAS_POS_VALID || data[7] < MAGBIAS_NEG_VALID) valid = false;
	if (data[8] > MAGBIAS_POS_VALID || data[8] < MAGBIAS_NEG_VALID) valid = false;
	
	//Mag Scale
	if (data[9] > MAGSCALE_POS_VALID || data[9] < MAGSCALE_NEG_VALID) valid = false;
	if (data[10] > MAGSCALE_POS_VALID || data[10] < MAGSCALE_NEG_VALID) valid = false;
	if (data[11] > MAGSCALE_POS_VALID || data[11] < MAGSCALE_NEG_VALID) valid = false;
	
	return (true);
}



bool adhrs::getWingsLevel(void)
{
	return hrs->getWingsLevel();
}

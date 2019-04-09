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

#include "airspeed.h"
#include "altitude.h"

static AHRS_CAL cal;
static AHRS_CAL defaultcal;
static AIR_DATAPOINT airDataBuffer[AIRDATABUFFERSIZE];
static float vsiBuffer[VSIDATABUFFERSIZE];
static int airDataBufferIndex;
static int vsiDataBufferIndex;
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
adhrs::adhrs(AHRS_CAL* pCalibration, bool domagtest, bool showmagvectors)
{	
	staticpress = new hsc_pressure();
	staticpress->set_params(pCalibration->staticPressMax, pCalibration->staticPressMin);
	airspeed = new hsc_pressure(1);
	airspeed->set_params(pCalibration->airspeedPressMax, pCalibration->airspeedPressMin);	

	hrs = new mpudriver(pCalibration->gyroBias, pCalibration->accelBias, 
		pCalibration->magBias, pCalibration->magScale, pCalibration->axisremap);				
	performMagTest = domagtest;
	showMagVectors = showmagvectors;
	airDataBufferIndex = 0;
	vsiDataBufferIndex = 0;
}

void adhrs::Init(void)
{
	int s = hrs->Init(true, false, performMagTest, showMagVectors);		
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
			AIR_DATAPOINT point = adhrs::processAirData(staticpress->getPressure(), airspeed->getPressure(), altimeterSetting);
			data->altitude = point.altitude;
			data->vsi = adhrs::processVSI();
			data->airspeed = point.speed;
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

AC_STATE adhrs::getAcState(void)
{
	return hrs->getAcState();
}


void adhrs::getCalibration(AHRS_CAL* calibration)
{	
	calibration = pCal;
}

void adhrs::setAltimeterSetting(int setting, int settingPrec)
{
	altimeterSetting = setting/pow(10, settingPrec);
}

/**********************************************************************************************//**
* Module: adhrs::calfile_process_line
***************************************************************************************************
* @brief	Static funciton used for processing lines out of an adhrs  cal file
* @note		The validation is "interesting" - first we invalidate everything. Then if it exists in 
*			the calfile, we validate it, then if it's out of bounds we invalidate it again. I didn't
*			want to add another flag to say if was in the file or not, and didn't want to invalidate
*			everything based on value alone
* @todo		
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 03/15/19 - Created & Commented
**************************************************************************************************/
void adhrs::calfile_process_line(QByteArray &line)
{
	if (line.startsWith("Gyro Offset X"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.gyroBias[0] = val;
		cal.gyroValid = true;
	}
	if (line.startsWith("Gyro Offset Y"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.gyroBias[1] = val;
		cal.gyroValid = true;
	}
	if (line.startsWith("Gyro Offset Z"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.gyroBias[2] = val;
		cal.gyroValid = true;
	}
	if (line.startsWith("Accel Offset X"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.accelBias[0] = val;
		cal.accelValid = true;
	}
	if (line.startsWith("Accel Offset Y"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.accelBias[1] = val;
		cal.accelValid = true;
	}
	if (line.startsWith("Accel Offset Z"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.accelBias[2] = val;
		cal.accelValid = true;
	}
	
	if (line.startsWith("Mag Offset X"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.magBias[0] = val;
		cal.magBiasValid = true;
	}
	if (line.startsWith("Mag Offset Y"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.magBias[1] = val;
		cal.magBiasValid = true;
	}
	if (line.startsWith("Mag Offset Z"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.magBias[2] = val;
		cal.magBiasValid = true;
	}
	if (line.startsWith("Mag Scale X"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.magScale[0] = val;
		cal.magScaleValid = true;
	}
	if (line.startsWith("Mag Scale Y"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.magScale[1] = val;
		cal.magScaleValid = true;
	}
	if (line.startsWith("Mag Scale Z"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.magScale[2] = val;	
		cal.magScaleValid = true;
	}
	if (line.startsWith("X axis Mapping"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.axisremap[0] = val;
		cal.axisRemapValid = true;
	}
	if (line.startsWith("Y axis Mapping"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.axisremap[1] = val;
		cal.axisRemapValid = true;
	}
	if (line.startsWith("Z axis Mapping"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.axisremap[2] = val;
		cal.axisRemapValid = true;
	}
	if (line.startsWith("X axis Gryo Sign"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.axisremap[3] = val;
		cal.axisRemapValid = true;
	}
	if (line.startsWith("X axis Accel Sign"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.axisremap[4] = val;
		cal.axisRemapValid = true;
	}
	if (line.startsWith("X axis Mag Sign"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.axisremap[5] = val;
		cal.axisRemapValid = true;
	}
	if (line.startsWith("Y axis Gryo Sign"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.axisremap[6] = val;
		cal.axisRemapValid = true;
	}
	if (line.startsWith("Y axis Accel Sign"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.axisremap[7] = val;
		cal.axisRemapValid = true;
	}
	if (line.startsWith("Y axis Mag Sign"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.axisremap[8] = val;
		cal.axisRemapValid = true;
	}
	if (line.startsWith("Z axis Gryo Sign"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.axisremap[9] = val;
		cal.axisRemapValid = true;
	}
	if (line.startsWith("Z axis Accel Sign"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.axisremap[10] = val;
		cal.axisRemapValid = true;
	}
	if (line.startsWith("Z axis Mag Sign"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.axisremap[11] = val;
		cal.axisRemapValid = true;
	}
	if (line.startsWith("Static Min Press"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.staticPressMin = val;
		cal.staticPressValid = true;
	}
	if (line.startsWith("Static Max Press"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.staticPressMax = val;
		cal.staticPressValid = true;
	}
	if (line.startsWith("Airspeed Min Press"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.airspeedPressMin = val;
		cal.airspeedPressValid = true;
	}
	if (line.startsWith("Airspeed Max Press"))
	{		
		float val = line.split('"')[1].toFloat();
		cal.airspeedPressMax = val;
		cal.airspeedPressValid = true;
	}

}

/**********************************************************************************************//**
* Module: adhrs::calfile_validate
***************************************************************************************************
* @brief	Static funciton used for validating an ahdrs calibration file
* @note		
* @todo		
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 03/15/19 - Created & Commented
**************************************************************************************************/
bool adhrs::calfile_validate()
{
	//Gyro
	for(int i = 0 ; i < 3 && cal.gyroValid ; i++)
	{
		if (cal.gyroBias[i] > GYRO_POS_VALID || cal.gyroBias[i] < GYRO_NEG_VALID) cal.gyroValid = false;
	}
	if (!cal.gyroValid)qDebug() << "gyro data not valid";
	
	//Accel
	for(int i = 0 ; i < 3 && cal.accelValid ; i++)
	{
		if (cal.accelBias[i] > ACCEL_POS_VALID || cal.accelBias[i] < ACCEL_NEG_VALID) cal.accelValid = false;
	}
	if (!cal.accelValid)qDebug() << "accel data not valid";	
	
	//Mag Offset
	for(int i = 0 ; i < 3 && cal.magBiasValid ; i++)
	{
		if (cal.magBias[i] > MAGBIAS_POS_VALID || cal.magBias[i] < MAGBIAS_NEG_VALID) cal.magBiasValid = false;
	}
	if (!cal.magBiasValid)qDebug() << "mag bias not valid";
	
	//Mag Scale
	for(int i = 0 ; i < 3 && cal.magScaleValid ; i++)
	{
		if (cal.magScale[i] > MAGSCALE_POS_VALID || cal.magScale[i] < MAGSCALE_NEG_VALID) cal.magScaleValid = false;
	}
	if (!cal.magScaleValid)qDebug() << "mag scale not valid";
		
	//Axis Remap
	for(int i = 0 ; i < 12 && cal.axisRemapValid ; i++)
	{
		if (cal.axisremap[i] > AXISREMAP_POS_VALID || cal.axisremap[i] < AXISREMAP_NEG_VALID) cal.axisRemapValid = false;
	}
	if (!cal.axisRemapValid) qDebug() << "axis remap not valid";
	
	//Static Pressure	
	if(cal.staticPressMax > PRESSURE_MAX_VALID || cal.staticPressMin < PRESSURE_MIN_VALID || cal.staticPressMax < cal.staticPressMin) cal.staticPressValid = false;
	
	if (!cal.staticPressValid) qDebug() << "static pressure cal not valid";
	
	//Airspeed Pressure	
	if(cal.airspeedPressMax > PRESSURE_MAX_VALID || cal.airspeedPressMin < PRESSURE_MIN_VALID || cal.airspeedPressMax < cal.airspeedPressMin) cal.airspeedPressValid = false;
	
	if (!cal.airspeedPressValid) qDebug() << "airspeed pressure cal not valid";
	
	return (cal.gyroValid || cal.accelValid || cal.magBiasValid || cal.magScaleValid || cal.axisRemapValid);
}

/**********************************************************************************************//**
* Module: adhrs::processCalibrationFile
***************************************************************************************************
* @brief	Static funciton used for loading a cal file to hand to this class on init.
* @note		
* @todo		
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 03/15/19 - Created & Commented
**************************************************************************************************/
AHRS_CAL* adhrs::processCalibrationFile(QFile* file)
{
	AHRS_CAL* pCal = &defaultcal;
	
	bool success = false;
	//invalidate the calibration
	cal.gyroValid = false;
	cal.accelValid = false;
	cal.magBiasValid = false;
	cal.magScaleValid = false;
	cal.axisRemapValid = false;
	cal.staticPressValid = false;
	cal.airspeedPressValid = false;
	
	adhrs::default_calibration(&cal);
	adhrs::default_calibration(&defaultcal);
	
	if (file != NULL)
	{	
		if (file->exists())
		{
			qDebug() << "adhrs calfile exists - processing..." << file->fileName();
			if (file->open(QIODevice::ReadOnly))
			{
				while (!file->atEnd()) {
					QByteArray line = file->readLine();
					calfile_process_line(line);
				}
				/* do we calibrate?*/
				qDebug() << "valid calfile loaded";
				if (calfile_validate())
				{
					//If anything in the file is valid, we'll use the cal which may or may not be mostly defaulted
					pCal = &cal;
					success = true;
				}
				file->close();
			}			
		}
	}
	if (!success) qDebug() << "***USING DEFAULT CALIBRATION***";
	return pCal;
}

/**********************************************************************************************//**
* Module: adhrs::processCalibrationFile
***************************************************************************************************
* @brief	incase we don't get a valid calfile, use these values
* @note		
* @todo		
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 03/15/19 - Created & Commented
**************************************************************************************************/
void adhrs::default_calibration(AHRS_CAL* cal)
{	
	cal->gyroBias[0] = 0.0;
	cal->gyroBias[1] = 0.0;
	cal->gyroBias[2] = 0.0;	
	
	cal->accelBias[0] = 0;
	cal->accelBias[1] = 0;
	cal->accelBias[2] = 0;

	cal->magBias[0] = 0;
	cal->magBias[1] = 0;
	cal->magBias[2] = 0;
	
	cal->magScale[0] = 1;
	cal->magScale[1] = 1;
	cal->magScale[2] = 1;	
	
	cal->axisremap[0] = 2.0;    //zaxis on imu is x axis for EFIS
	cal->axisremap[1] = 1.1;     //yaxis on imu is y axis for EFIS
	cal->axisremap[2] = 0.1;     //xaxis on imu is z axis for EFIS
	cal->axisremap[3] = 1.0;     //xaxis gyro for EFIS is not inverted
	cal->axisremap[4] = 1.0;      //xaxis accel for EFIS is not inverted
	cal->axisremap[5] = 1.0;       //xaxis mag for EFIS is not inverted
	cal->axisremap[6] = -1.0;      //yaxis gyro for EFIS is inverted
	cal->axisremap[7] = 1.0;       //yaxis accel for EFIS is not inverted
	cal->axisremap[8] = 1.0;       //yaxis mag for EFIS is not inverted
	cal->axisremap[9] = -1.0;      //zaxis gyro for EFIS is not inverted
	cal->axisremap[10] = 1.0;       //zaxis accel for EFIS is not inverted
	cal->axisremap[11] = 1.0;       //zaxis mag for EFIS is not inverted	
	
	cal->airspeedPressMax = 1.0f;
	cal->airspeedPressMin = -1.0f;
	cal->staticPressMax = 15.0f;
	cal->staticPressMin = 0.0f;

}



void adhrs::setSteerToSettings(int* steerto)
{
	hrs->setSteerCard(steerto);
}


void adhrs::setAttitudeOffset(int offset_deg)
{
	//Note the negative here to account for the opposite of the level flag in the AHRS
	hrs->setAttitudeOffset((float)(-offset_deg*M_PI/180.0f));
}


AIR_DATAPOINT adhrs::processAirData(float pStatic, float pPitot, float altSetting)
{
	AIR_DATAPOINT point;
	
	timeval tp;
	int v = gettimeofday(&tp, NULL);	
	
	int airDataBufferIndexPrev = (airDataBufferIndex - 1) >= 0 ? airDataBufferIndex - 1 : AIRDATABUFFERSIZE;
	
	
	airDataBuffer[airDataBufferIndex].altitude = altitude::getAltitudeFt(pStatic, altSetting);
	airDataBuffer[airDataBufferIndex].speed = airspeed::getIASMph(pPitot);
	airDataBuffer[airDataBufferIndex].seconds = tp.tv_sec;
	airDataBuffer[airDataBufferIndex].useconds = tp.tv_usec;
	
	int secs = airDataBuffer[airDataBufferIndex].seconds - airDataBuffer[airDataBufferIndexPrev].seconds;
	float _dt = secs * 1000000.0f + (airDataBuffer[airDataBufferIndex].useconds - airDataBuffer[airDataBufferIndexPrev].useconds);
	_dt /= 60000000.0f;	
		
	float altdiff = airDataBuffer[airDataBufferIndex].altitude - airDataBuffer[airDataBufferIndexPrev].altitude;
		
	airDataBuffer[airDataBufferIndex].vsi = (altdiff / _dt);		
	
	
	point.altitude = airDataBuffer[airDataBufferIndex].altitude;
	point.speed = airDataBuffer[airDataBufferIndex].speed;	
	
	airDataBufferIndex++;
	if (airDataBufferIndex >= AIRDATABUFFERSIZE) {
		airDataBufferIndex = 0;
	}
	
	return point;
}


float adhrs::processVSI(void)
{
	float vsi = 0;
	for (int i = 0; i < AIRDATABUFFERSIZE; i++)
	{
		vsi += airDataBuffer[i].vsi;
	}
	
	vsi /= (AIRDATABUFFERSIZE);
	
	vsiBuffer[vsiDataBufferIndex] = vsi;
	
	vsiDataBufferIndex++;
	if (vsiDataBufferIndex >= VSIDATABUFFERSIZE) vsiDataBufferIndex = 0;
	
	vsi = 0;
	for (int i = 0; i < VSIDATABUFFERSIZE; i++)
	{
		vsi += vsiBuffer[i];
	}
	vsi /= VSIDATABUFFERSIZE;
		
	return vsi;
}

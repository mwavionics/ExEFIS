#pragma once
#include "hsc_pressure.h"
#include <QByteArray>
#include <QTimer>
#include "mpudriver.h"

//Defines for validity checks on the calibraitons
#define GYRO_POS_VALID 1.0f
#define GYRO_NEG_VALID -1.0f
#define ACCEL_POS_VALID 0.1f
#define ACCEL_NEG_VALID -0.1f
#define MAGBIAS_POS_VALID 500.0f
#define MAGBIAS_NEG_VALID -500.0f
#define MAGSCALE_POS_VALID 1.2f
#define MAGSCALE_NEG_VALID -1.2f

class adhrs
{
public:
	adhrs();
	~adhrs();
	void readAll(void);
	int getAllSixRaw(float* data);	
	int getOffsets(char* calData);
	int setOffsets(char* calData);
	void getCalibration(char* cal);
	
	
private:
	float caldata[12];
	hsc_pressure *staticpress;
	hsc_pressure *airspeed;
	float staticPressurePSI;
	float aspPressurePSI;
	float euHeading; 
	float euRoll;	
	float euPitch;	
	float slipRAW;
	void calfile_process_line(QByteArray &line, float* data);
	bool calfile_validate(float* data);
	mpudriver *hrs;
	
};


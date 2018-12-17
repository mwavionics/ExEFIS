#pragma once
#include "hsc_pressure.h"
#include <QByteArray>
#include <QTimer>
#include "mpudriver.h"

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


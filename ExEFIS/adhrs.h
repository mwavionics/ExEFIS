#pragma once
#include "hsc_pressure.h"
#include <QByteArray>
#include <QTimer>
#include "mpudriver.h"

//Defines for validity checks on the calibraitons
#define GYRO_POS_VALID 100.0f
#define GYRO_NEG_VALID -100.0f
#define ACCEL_POS_VALID 1.0f
#define ACCEL_NEG_VALID -1.0f
#define MAGBIAS_POS_VALID 500.0f
#define MAGBIAS_NEG_VALID -500.0f
#define MAGSCALE_POS_VALID 1.5f
#define MAGSCALE_NEG_VALID -1.5f
#define AXISREMAP_POS_VALID 3.0f
#define AXISREMAP_NEG_VALID -2.0f

typedef struct
{
	float staticPressurePSI;
	float aspPressurePSI;
	float heading;
	float roll;
	float pitch;
	float slip;
}AHRS_DATA;

class adhrs
{
public:
	adhrs(bool domagtest, bool showmagvectors);
	~adhrs();

	int getDataSet(AHRS_DATA* data);	
	int getOffsets(char* calData);
	int setOffsets(char* calData);
	void getCalibration(char* cal);
	bool getWingsLevel(void);
	
	
private:
	float caldata[24];
	hsc_pressure *staticpress;
	hsc_pressure *airspeed;
	mpudriver *hrs;
	
	void calfile_process_line(QByteArray &line, float* data);
	bool calfile_validate(float* data, volatile float* pGyroBias, volatile float* pAccelBias, 
		volatile float* pMagBias,
		volatile float* pMagScale, volatile float* pAxisRemap);
	
	
};


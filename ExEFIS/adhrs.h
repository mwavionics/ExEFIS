#pragma once
#include <QByteArray>
#include <QTimer>
#include <QFile>

#include "mpudriver.h"
#include "hsc_pressure.h"
#include "SKFilter.h"

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
#define PRESSURE_MAX_VALID 30.0f
#define PRESSURE_MIN_VALID -30.0f

//Define the buffer size for the altimeter and VSI
#define ALTBUFFERSIZE 50
#define VSIBUFFERSIZE 50


typedef struct
{
	float altitude;
	float vsi;
	float airspeed;
	float heading;
	float roll;
	float pitch;
	float slip;
}AHRS_DATA;

typedef struct
{
	float speed;
	float altitude;
	int seconds;
	int useconds;
}AIR_DATAPOINT;

typedef struct
{
	float gyroBias[3];
	bool gyroValid;
	
	float accelBias[3];
	bool accelValid;
	
	float magBias[3];
	bool magBiasValid;
	
	float magScale[3];
	bool magScaleValid;
	
	float axisremap[12];
	bool axisRemapValid;
	
	float staticPressMax;
	float staticPressMin;
	bool staticPressValid;
	
	float airspeedPressMax;
	float airspeedPressMin;
	bool airspeedPressValid;
}AHRS_CAL;

class adhrs
{
public:
	adhrs(AHRS_CAL* pCalibration, bool domagtest, bool showmagvectors);
	~adhrs();
	
	void Init(void);

	int getDataSet(AHRS_DATA* data);
	void getCalibration(AHRS_CAL* cal);
	AC_STATE getAcState(void);		
	void setAltimeterSetting(int setting, int settingPrec);
	void setSteerToSettings(int* steerto);
	static AHRS_CAL* processCalibrationFile(QFile* file);

	
	
private:
	hsc_pressure *staticpress;
	hsc_pressure *airspeed;
	mpudriver *hrs;
	AHRS_CAL* pCal;
	float altimeterSetting;
	bool performMagTest = false;
	bool showMagVectors = false;
	
	static void default_calibration(AHRS_CAL* cal);
	static void calfile_process_line(QByteArray &line);
	static bool calfile_validate( void );
	
	AIR_DATAPOINT altbuffer[ALTBUFFERSIZE];	
	int altbufferindex;
	//int vsibufferindex;
	
	
};


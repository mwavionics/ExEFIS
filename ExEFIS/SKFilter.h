#pragma once
#include "Vector.h"
#include <sys/time.h>
#include <math.h>

typedef struct 
{
	bool magpitch;
	bool magyaw;
	bool magroll;
	bool gyropitch;
	bool gyroyaw;
	bool gyroroll;
	bool accelpitch;
	bool accelyaw;
	bool accelroll;
} AXIS_STILL;

typedef struct
{
	bool wingslevel;
	bool pitchlevel;
	bool pitchstill;
	bool turning;
	bool ballcentered;
	bool oneG;
} AC_STATE;

class SKFilter
{
	
	public:
	SKFilter();
	~SKFilter();
	void setInitializationDuration(int duration);
	bool validate(float gx, float gy, float gz, float ax, float ay, float az, float hx, float hy, float hz);
	bool update(float gx, float gy, float gz, float ax, float ay, float az, float hx, float hy, float hz);
	
	void setAttitudeOffset(float offset_rads);

	imu::Vector<3> getEuler(void);	
	AC_STATE acState;
	imu::Vector<3> dmag;	
	AXIS_STILL axisstill;
	//	
	float forSteerTable[13] = {0.0f, M_PI / 6, M_PI / 3, M_PI / 2, 2*M_PI / 3, 5*M_PI / 6, M_PI, 7*M_PI / 6, 4*M_PI / 3, 3*M_PI / 2, 5*M_PI / 3, 11*M_PI / 6, 2*M_PI };
	float steerTable[13] = {0.0f, M_PI / 6, M_PI / 3, M_PI / 2, 2*M_PI / 3, 5*M_PI / 6, M_PI, 7*M_PI / 6, 4*M_PI / 3, 3*M_PI / 2, 5*M_PI / 3, 11*M_PI / 6, 2*M_PI };
	
private:
	bool _initialized = false;
	int _initCounter = 0;
	// timing
	float _tnow, _tprev, _dt;
	float valid;
	timeval last;
	
	float gyroHeading;
	
	int magbufferindex = 0;
	int accelbufferindex = 0;
	int eulerbufferindex = 0;
	
	
	
	// store previous sensor values to see when updated
	float gx_, gy_, gz_;
	float ax_, ay_, az_;
	float mx_, my_, mz_;
	float dmx, dmy, dmz;
	
	bool gyroUpdated_, accelUpdated_, magUpdated_;
	
	float attitudeOffset;
	
	float constrainAngle180(float dta);
	float constrainAngle360(float dta);
	
	imu::Vector<3> _Euler;
	imu::Vector<3> _Euler_Fixed;
	
	imu::Vector<3> _gyroEuler;
	imu::Vector<3> _magEuler;
	imu::Vector<3> _accelEuler;
		
	imu::Vector<3> gyroPrev;
	imu::Vector<3> magPrev;
	imu::Vector<3> accelPrev;	
};


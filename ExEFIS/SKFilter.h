#pragma once
#include "Vector.h"
#include <sys/time.h>


class SKFilter
{
public:
	SKFilter();
	~SKFilter();
	void setInitializationDuration(int duration);
	bool validate(float gx, float gy, float gz, float ax, float ay, float az, float hx, float hy, float hz);
	bool update(float gx, float gy, float gz, float ax, float ay, float az, float hx, float hy, float hz);
	float getRoll_rad();
	float getPitch_rad();
	float getYaw_rad();
	float getHeading_rad();
	int quad = 0;
	imu::Vector<3> getEuler(void);
private:
	bool _initialized = false;
	int _initCounter = 0;
	// timing
	float _tnow, _tprev, _dt;
	float valid;
	timeval last;
	
	float magHeading;
	
	int magbufferindex = 0;
	
	
	
	// store previous sensor values to see when updated
	float gx_, gy_, gz_;
	float ax_, ay_, az_;
	float mx_, my_, mz_;
	
	bool gyroUpdated_, accelUpdated_, magUpdated_;
	
	float constrainAngle180(float dta);
	float constrainAngle360(float dta);
	
	imu::Vector<3> _Euler;
	imu::Vector<3> _Euler_Fixed;
	imu::Vector<3> _magEuler;
	imu::Vector<3> magPrev;
	imu::Vector<3> gyroPrev;
	imu::Vector<3> _InitialEuler;
	
	
};


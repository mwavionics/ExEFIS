#include "SKFilter.h"
#include "Vector.h"
#include "math.h"
#include <QDebug>

const int magbuffersize = 100;
static float magbuffer[3][magbuffersize];


SKFilter::SKFilter()
{
	valid = false;
	
	_Euler[0] = 0;
	_Euler[1] = 0;
	_Euler[2] = M_PI;
	
	//Reference Coordinates for the chip, not the airplane
	_Euler_Fixed[0] = 0;
	_Euler_Fixed[1] = 0;
	_Euler_Fixed[2] = M_PI;	
	
	gyroPrev[0] = 0;
	gyroPrev[1] = 0;
	gyroPrev[2] = 0;
	
	magHeading = 0;
	_initCounter = 0;
	_initialized = false;
	magbufferindex = 0;
}




SKFilter::~SKFilter()
{
}

//This does nothing right now
void SKFilter::setInitializationDuration(int duration)
{
	this->_initialized = false;	
}

bool SKFilter::validate(float gx, float gy, float gz, float ax, float ay, float az, float hx, float hy, float hz)
{
	valid = true;
	
	if (ax > 6.0 || ax < -6.0 || 
		ay > 6.0 || ay < -6.0 || 
		az > 6.0 || az < -6.0) valid = false;
	
	if (gx > 400 || gx < -400 ||
		gy > 400 || gy < -400 ||
		gz > 400 || gz < -400) valid = false;
	
	/* take the time sample now so when we get another value we're not hosed*/
	static timeval last;
		
	timeval tp;
	int v = gettimeofday(&tp, NULL);
	int secs = tp.tv_sec - last.tv_sec;
	_dt = secs * 1000000.0f + (tp.tv_usec - last.tv_usec);
	_dt /= 1000000.0f;
	last = tp;
	
	if (_dt > 0.2) valid = false;
	
	if (!valid)
	{
		
		/* Reset the Previous values*/
		gyroPrev[0] = 0;
		gyroPrev[1] = 0;
		gyroPrev[2] = 0;
	
	}
	
	return valid;
}


bool SKFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	if (valid)
	{
		
	
		// checking if gyro data updated
	 if((gx != gx_) || (gy != gy_) || (gz != gz_)) {
			gyroUpdated_ = true;
			gyroPrev[0] = gx_;
			gx_ = gx;
			gyroPrev[1] = gy_;
			gy_ = gy;
			gyroPrev[2] = gz_;
			gz_ = gz;
		} else {
			gyroUpdated_ = false;
		}

		// checking if accel data updated
		if((ax != ax_) || (ay != ay_) || (az != az_)) {
			accelUpdated_ = true;
			ax_ = ax;
			ay_ = ay;
			az_ = az;
		} else {
			accelUpdated_ = false;
		}

		// checking if magnetometer data updated
		if((mx != mx_) || (my != my_) || (mz != mz_)) {
			magUpdated_ = true;
			magbuffer[0][magbufferindex] = mx;
			magbuffer[1][magbufferindex] = my;
			magbuffer[2][magbufferindex] = mz;
			magbufferindex++;
			if (magbufferindex > magbuffersize) magbufferindex = 0;
			mx_ = 0;
			my_ = 0;
			mz_ = 0;
			for (int i = 0; i < magbuffersize; i++)
			{
				mx_ += magbuffer[0][i];
				my_ += magbuffer[1][i];
				mz_ += magbuffer[2][i];
			}
			mx_ /= magbuffersize;
			my_ /= magbuffersize;
			mz_ /= magbuffersize;
		
		} else {
			magUpdated_ = false;
		}
	
		if (gyroUpdated_ || magUpdated_ || accelUpdated_)
		{
			_initCounter++;
			if (_initCounter > 100) _initialized = true;
			// get the change in time
			//_tnow = (float) micros() / 1000000.0;
			//_dt = _tnow - _tprev;
			//_tprev = _tnow;
		}
	
	
		if (_initialized)
		{	
			//Check if the gyro has been updated
			if (gyroUpdated_) {	
		
				/* Calc Integral */
				imu::Vector<3> integral;
				integral[0] = ((gyroPrev[0] * _dt) + ((gx_ - gyroPrev[0]) * 0.5f * _dt));
				integral[1] = ((gyroPrev[1] * _dt) + ((gy_ - gyroPrev[1]) * 0.5f * _dt));
				integral[2] = ((gyroPrev[2] * _dt) + ((gz_ - gyroPrev[2]) * 0.5f * _dt));
				
				//First Calc Roll Angle
				_Euler[0] += integral[0];
				if (_Euler[0] < (-2*M_PI)) _Euler[0] += 2*M_PI;
				if (_Euler[0] > (2*M_PI)) _Euler[0] -= 2*M_PI;
			
				//Now, apply the pitch component and yaw component correctly to pitch if you're banked, same for yaw
				_Euler[1] += (integral[1] * cos(_Euler[0])) + (integral[2] * sin(_Euler[0]));
				magHeading += (integral[2] * cos(_Euler[0])) + (integral[1] * sin(_Euler[0]));
			
				_Euler[2] = constrainAngle360(magHeading);
				//_Euler[2] += integral[2];
				
				if(_Euler[1] < (-2*M_PI)) _Euler[1] += 2*M_PI;
				if (_Euler[1] > (2*M_PI)) _Euler[1] -= 2*M_PI;
			}
	
			//Hack to pull back to center like spring in mech DG
			if(accelUpdated_)
			{
				if (abs(az_ - 1.0f) < 0.05f)
				{
					if (_Euler[0] > 0) _Euler[0] -= 0.05f * _dt;   //move at 0.1 rad / s
					else _Euler[0] += 0.1f * _dt;
					if (_Euler[1] > 0) _Euler[1] -= 0.05f * _dt;     //move at 0.1 rad / s
					else _Euler[1] += _dt;
				}
				//_Euler[0] = constrainAngle360(_Euler[0]);
				//_Euler[1] = constrainAngle360(_Euler[1]);
			}
	
			if (magUpdated_)
			{
				float truemz = (sin(_Euler[0]) * my_) + (cos(_Euler[0])* mz_);
				float truemx = (sin(_Euler[1]) * my_) + (cos(_Euler[1])* mx_);
				
				//x to front of plane y to wing
				float tan = truemz / truemx;
				float rads = atan(tan);
				//which quadrant are we in?
				float quadrantOffset = 0;
		
				if (truemz >= 0 && truemx >= 0) quad = 0;    //quadrantOffset = 0;
				if(truemz >= 0 && truemx <  0) quad = 1;     //quadrantOffset = 180 *M_PI / 180.0f;
				if(truemz <  0 && truemx <  0) quad = 2;     //quadrantOffset = 180 *M_PI / 180.0f;
				if(truemz <  0 && truemx >= 0) quad = 3;     //quadrantOffset = 0 *M_PI / 180.0f;
		
				if(truemz >= 0 && truemx >= 0) quadrantOffset = 0;
				if (truemz >= 0 && truemx <  0)quadrantOffset = M_PI;
				if (truemz <  0 && truemx <  0) quadrantOffset = M_PI;
				if (truemz <  0 && truemx >= 0) quadrantOffset = 2*M_PI;
		
				float gyroHeading = constrainAngle360(magHeading);
				float magCalc = rads + quadrantOffset;
		
				float diff = magCalc - gyroHeading;
		
				//magHeading += (diff / 2); //SSK killed magentic for now
		
				//_Euler[2] = constrainAngle360(magHeading);
			}
		}
	}
	
}

/* Returns the roll angle, rad */
float SKFilter::getRoll_rad() {
	return _Euler.x();
}

/* Returns the pitch angle, rad */
float SKFilter::getPitch_rad() {
	return _Euler.y();
}

/* Returns the yaw angle, rad */
float SKFilter::getYaw_rad() {
	return constrainAngle180(_Euler.z() - _InitialEuler.z());
}

/* Returns the heading angle, rad */
float SKFilter::getHeading_rad() {
	return _Euler.z();//constrainAngle360(_Euler.z());
}

imu::Vector<3> SKFilter::getEuler(void)
{
	return (this->_Euler);
}


/* Bound angle between -180 and 180 */
float SKFilter::constrainAngle180(float dta) {
	if (dta >  M_PI) dta -= (M_PI * 2.0);
	if (dta < -M_PI) dta += (M_PI * 2.0);
	return dta;
}

/* Bound angle between 0 and 360 */
float SKFilter::constrainAngle360(float dta) {
	dta = fmod(dta, 2.0*M_PI);
	if (dta < 0.0)
		dta += 2.0*M_PI;
	return dta;
}


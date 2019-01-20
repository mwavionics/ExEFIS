#include "SKFilter.h"
#include "Vector.h"
#include "math.h"
#include <QDebug>

const int magbuffersize = 500;
static float magbuffer[3][magbuffersize];
static float dmagbuffer[3][magbuffersize];
static timeval last;

/**********************************************************************************************//**
* Module: SKFilter::SKFilter
***************************************************************************************************
* @brief	init the SK Filter for AHRS

* @note		
*
* @todo		
*	
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 11/06/17 - Created & Commented
**************************************************************************************************/
SKFilter::SKFilter()
{
	valid = false;
	
	_Euler[0] = 0;
	_Euler[1] = 0;
	_Euler[2] = M_PI;
	
	//Reference Coordinates for the chip, not the airplane
	_Euler_Fixed[0] = 0;
	_Euler_Fixed[1] = 0;
	_Euler_Fixed[2] = 0;	
	
	_magEuler[0] = 0;
	_magEuler[1] = 0;
	_magEuler[2] = 0;
	
	_gyroEuler[0] = 0;
	_gyroEuler[1] = 0;
	_gyroEuler[2] = 0;
	
	gyroPrev[0] = 0;
	gyroPrev[1] = 0;
	gyroPrev[2] = 0;
	
	magPrev[0] = 0;
	magPrev[1] = 0;
	magPrev[2] = 0;
	
	magHeading = 0;
	_initCounter = 0;
	_initialized = false;
	wingslevel = false;
	magbufferindex = 0;
}

/**********************************************************************************************//**
* Module: SKFilter::~SKFilter
***************************************************************************************************
* @brief	Deconstructor for the SK Filter AHRS

* @note		
*
* @todo		
*	
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 11/06/17 - Created & Commented
**************************************************************************************************/
SKFilter::~SKFilter()
{
}

//This does nothing right now
void SKFilter::setInitializationDuration(int duration)
{
	this->_initialized = false;	
}

/**********************************************************************************************//**
* Module: SKFilter::validate
***************************************************************************************************
* @brief	Validate the accel, gyro, and magnetometer readings beofer processing

* @note		used to filter out HF noise from gyro, etc...
*
* @todo		
*	
* @returns	true if data is valid
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 11/06/17 - Created & Commented
**************************************************************************************************/
bool SKFilter::validate(float gx, float gy, float gz, float ax, float ay, float az, float hx, float hy, float hz)
{
	valid = true;
	
	// Bound valid accel values to +/- 6 g's
	if (ax > 6.0 || ax < -6.0 || 
		ay > 6.0 || ay < -6.0 || 
		az > 6.0 || az < -6.0) valid = false;
	
	//Bound gyro values to +/- 360 deg/sec
	if (gx > 2*M_PI || gx < -2*M_PI ||
		gy > 2*M_PI || gy < -2*M_PI ||
		gz > 2*M_PI || gz < -2*M_PI) valid = false;
	
	/* take the time sample now so when we get another value we're not hosed*/		
	timeval tp;
	int v = gettimeofday(&tp, NULL);
	int secs = tp.tv_sec - last.tv_sec;
	_dt = secs * 1000000.0f + (tp.tv_usec - last.tv_usec);
	_dt /= 1000000.0f;	
	
	if (_dt > 0.2)
	{
		printf("invalid dt, > 0.2, tp.tv_sec = %d tp.tv_usec = %d last.tv_sec = %d last.tv_usec = %d \n", tp.tv_sec, tp.tv_usec, last.tv_sec, last.tv_usec );
		valid = false;
	}
	if (_dt < 0.0f)
	{
		printf("invalid dt, < 0.0, tp.tv_sec = %d tp.tv_usec = %d last.tv_sec = %d last.tv_usec = %d \n", tp.tv_sec, tp.tv_usec, last.tv_sec, last.tv_usec);
		valid = false;
	}
	
	if (!valid)
	{
		/* Reset the Previous values*/
		gyroPrev[0] = 0;
		gyroPrev[1] = 0;
		gyroPrev[2] = 0;	
	}
	
	last.tv_sec = tp.tv_sec;
	last.tv_usec = tp.tv_usec;
	
	return valid;
}

/**********************************************************************************************//**
* Module: SKFilter::update
***************************************************************************************************
* @brief	update the orientation

* @note		
*
* @todo		
*	
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 11/06/17 - Created & Commented
**************************************************************************************************/
bool SKFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	if (valid)
	{
			
	
		// checking if gyro data updated
		if((gx != gx_) || (gy != gy_) || (gz != gz_)) 
		{
				gyroUpdated_ = true;
				gyroPrev[0] = gx_;
				gx_ = gx;
				gyroPrev[1] = gy_;
				gy_ = gy;
				gyroPrev[2] = gz_;
				gz_ = gz;
			} 
		else 
		{
				gyroUpdated_ = false;
		}

		// checking if accel data updated
		if((ax != ax_) || (ay != ay_) || (az != az_)) 
		{
			accelUpdated_ = true;
			ax_ = ax;
			ay_ = ay;
			az_ = az;
		} 
		else 
		{
			accelUpdated_ = false;
		}

		// checking if magnetometer data updated
		if((mx != mx_) || (my != my_) || (mz != mz_)) 
		{
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
		
		} 
		else 
		{
			magUpdated_ = false;
		}
	
		if (gyroUpdated_ || magUpdated_ || accelUpdated_)
		{
			_initCounter++;
			if (_initCounter > 100) _initialized = true;		
		}	
	
		if (_initialized)
		{	
			
			if (magUpdated_)
			{

				float mag_norm = sqrt((mx_*mx_) + (my_*my_) + (mz_*mz_));
				float xmag = mx_ / mag_norm;
				float ymag = my_ / mag_norm;
				float zmag = mz_ / mag_norm;
				
				//Adjust for bank angle and pitch
				float Roll = 0; //_Euler[0];
				float Pitch = 0; //_Euler[1];
				float Yaw = 0; //_Euler[2];
#if 0				
				float rollmag = atan2((-ymag*cos(Pitch) + xmag*sin(Pitch)), (zmag*cos(Yaw) + ymag*sin(Yaw)*sin(Pitch) + xmag*sin(Yaw)*cos(Pitch))); 
				_magEuler[0] = constrainAngle180(rollmag - M_PI);  //need to offset this by pi
			//	Roll = _magEuler[0];
				
				float pitchmag = -atan2((-xmag*cos(Yaw) + ymag*sin(Yaw)), (zmag*cos(Roll) + xmag*sin(Roll)*sin(Yaw) + ymag*sin(Roll)*cos(Yaw))); 
				_magEuler[1] = constrainAngle180(pitchmag - M_PI);  //need to offset this by pi
			//	Pitch = _magEuler[1];
				
				float yawmag = -atan2((-ymag*cos(Roll) + zmag*sin(Roll)), (xmag*cos(Pitch) + ymag*sin(Pitch)*sin(Roll) + zmag*sin(Pitch)*cos(Roll))); 
				_magEuler[2] = constrainAngle360(yawmag);
#endif				
				float rollmag = atan2((-ymag), (zmag)); 
				_magEuler[0] = constrainAngle180(rollmag - M_PI);   //need to offset this by pi
			//	Roll = _magEuler[0];
				
				float pitchmag = -atan2((-xmag ), (zmag)); 
				_magEuler[1] = constrainAngle180(pitchmag - M_PI);   //need to offset this by pi
			//	Pitch = _magEuler[1];
				
				float yawmag = -atan2((-ymag), (xmag)); 
				_magEuler[2] = constrainAngle360(yawmag);
				
				
				
				dmagbuffer[0][magbufferindex] = (_magEuler[0] - magPrev[0]) / _dt ;
				dmagbuffer[1][magbufferindex] = (_magEuler[1] - magPrev[1]) / _dt ;
				dmagbuffer[2][magbufferindex] = (_magEuler[2] - magPrev[2]) / _dt ;	
				dmag[0] = 0;
				dmag[1] = 0;
				dmag[2] = 0;			
			
				for (int i = 0; i < magbuffersize; i++)
				{			
				
					dmag[0] += dmagbuffer[0][i];
					dmag[1] += dmagbuffer[1][i];
					dmag[2] += dmagbuffer[2][i];
				}
				dmag[0] /= magbuffersize;
				dmag[1] /= magbuffersize;
				dmag[2] /= magbuffersize;
				
				
				
				if (abs(dmag[2]) < 0.02f)
				{
					magyawstill = true;
					//printf("yawstill");
				}
				else magyawstill = false;
				
#if 0
				imu::Vector<3> magintegral;
				magintegral[0] = ((magPrev[0] * _dt) + ((dmag[0] - magPrev[0]) * 0.5f * _dt));
				magintegral[1] = ((magPrev[1] * _dt) + ((dmag[1] - magPrev[1]) * 0.5f * _dt));
				magintegral[2] = ((magPrev[2] * _dt) + ((dmag[2] - magPrev[2]) * 0.5f * _dt));
				
				//First Calc Roll Angle
				_magEuler[0] += magintegral[0];
				_magEuler[0] = constrainAngle180(_Euler[0]);
				
				if (_magEuler[0] > 4*M_PI || _magEuler[0] < -4*M_PI) 
				{
					_magEuler[0] = 0;
					printf("Mag Euler[0] out of range dt = %2.2f gy = %2.2f gprev = %2.2f", _dt, gx_, magPrev[0]);
				}
				
				//Now, apply the pitch component and yaw component correctly to pitch if you're banked, same for yaw
				_magEuler[1] += (magintegral[1] * cos(_magEuler[0])) + (magintegral[2] * sin(_magEuler[0]));
				_magEuler[1] = constrainAngle180(_magEuler[1]);
				
				if (_magEuler[1] > 4*M_PI || _magEuler[1] < -4*M_PI) 
				{
					_magEuler[1] = 0;
					printf("MagEuler[1] out of range dt = %+2.2f gy = %+2.2f gprev = %2.2f", _dt, gy_, magPrev[1]);
				}

				magHeading += (magintegral[2] * cos(_magEuler[0])) + (magintegral[1] * sin(_magEuler[0]));			
				_magEuler[2] = constrainAngle360(magHeading);
				if (_magEuler[2] > 4*M_PI || _magEuler[2] < -4*M_PI) 
				{
					_magEuler[2] = 0;
					printf("MagEuler[2] out of range dt = %+2.2f gy = %+2.2f gprev = %2.2f", _dt, gz_, magPrev[2]);
				}
#endif
				magPrev[0] = _magEuler[0];
				magPrev[1] = _magEuler[1];
				magPrev[2] = _magEuler[2];
				
								
#if 0				//SK Junk				//Assumes no pitch or roll for combo...				float mag_norm = sqrt((mx_*mx_) + (my_*my_) + (mz_*mz_));
				
				float xnorm = sqrt((my*my) + (mz*mz));
				float rotx = asin(mz_ / xnorm);  //Euler[0] - roll
				//printf("zmagg = %2.3f xnorm = %2.3f \n", mz_, xnorm);
				_Euler[0] = rotx;
				
				float ynorm = sqrt((mx*mx) + (mz*mz));
				float roty = asin(mz_ / ynorm);  //Euler[1] - pitch
				_Euler[1] = roty;
				
				float znorm = sqrt((mx*mx) + (my*my));
				float rotz = asin(my_ / znorm);  //Euler[2] - yaw
				_Euler[2] = rotz;
				

				//flip y and z here and this reads mag pitch... works!!!
				float truemz = (sin(_Euler[0]) * mz_) + (cos(_Euler[0])* my_);
				float truemx = (sin(_Euler[1]) * mz_) + (cos(_Euler[1])* mx_);
				
				//x to front of plane y to wing
				float tan = truemz / truemx;
				float rads = atan(tan);
				//which quadrant are we in?
				float quadrantOffset = 0;
		
				if (truemz >= 0 && truemx >= 0) quad = 0;      //quadrantOffset = 0;
				if(truemz >= 0 && truemx <  0) quad = 1;       //quadrantOffset = 180 *M_PI / 180.0f;
				if(truemz <  0 && truemx <  0) quad = 2;       //quadrantOffset = 180 *M_PI / 180.0f;
				if(truemz <  0 && truemx >= 0) quad = 3;       //quadrantOffset = 0 *M_PI / 180.0f;
		
				if(truemz >= 0 && truemx >= 0) quadrantOffset = 0;
				if (truemz >= 0 && truemx <  0)quadrantOffset = M_PI;
				if (truemz <  0 && truemx <  0) quadrantOffset = M_PI;
				if (truemz <  0 && truemx >= 0) quadrantOffset = 2*M_PI;
		
				float gyroHeading = constrainAngle360(magHeading);
				float magCalc = rads + quadrantOffset;
		
				float diff = magCalc - gyroHeading;
		
				magHeading += (diff / 2);  //SSK killed magentic for now
		
				_Euler[2] = constrainAngle360(magCalc);
#endif
			}
			//Check if the gyro has been updated
			if (gyroUpdated_) {	

				//	float gxx = (gx_ < 5) ? dmag[0] : (0.2*dmag[0]) + (0.8*gx_);
				//	float gyy = (gy_ < 5) ? dmag[1] : (0.2*dmag[1]) + (0.8*gy_);
				//	float gzz = (gz_ < 5) ? dmag[2] : (0.2*dmag[2]) + (0.8*gz_);
				
				float gxx = gx_;
				float gyy = gy_;
				float gzz = gz_;
					
				/* Calc Integral */
				imu::Vector<3> integral;
				integral[0] = ((gyroPrev[0] * _dt) + ((gxx - gyroPrev[0]) * 0.5f * _dt));
				integral[1] = ((gyroPrev[1] * _dt) + ((gyy - gyroPrev[1]) * 0.5f * _dt));
				integral[2] = ((gyroPrev[2] * _dt) + ((gzz - gyroPrev[2]) * 0.5f * _dt));
				
				//First Calc Roll Angle
				_gyroEuler[0] += integral[0];
				_gyroEuler[0] = constrainAngle180(_gyroEuler[0]);
				
				if (_gyroEuler[0] > 4*M_PI || _gyroEuler[0] < -4*M_PI) 
				{
					_gyroEuler[0] = 0;
					printf("Euler[0] out of range dt = %2.2f gy = %2.2f gprev = %2.2f", _dt, gx_, gyroPrev[0]);
				}
				
				//Now, apply the pitch component and yaw component correctly to pitch if you're banked, same for yaw
				_gyroEuler[1] += (integral[1] * cos(_gyroEuler[0])) + (integral[2] * sin(_gyroEuler[0]));
				_gyroEuler[1] = constrainAngle180(_gyroEuler[1]);
				
				if (_gyroEuler[1] > 4*M_PI || _Euler[1] < -4*M_PI) 
				{
					_gyroEuler[1] = 0;
					printf("Euler[1] out of range dt = %+2.2f gy = %+2.2f gprev = %2.2f", _dt, gy_, gyroPrev[1]);
				}
				
				float mghdiff = (integral[2] * cos(_gyroEuler[0])) + (integral[1] * sin(_gyroEuler[0]));	
				magHeading += mghdiff; 			
				
				if (abs(mghdiff) < 0.25) gyroyawstill = true;
				else gyroyawstill = false;
								
				_gyroEuler[2] = constrainAngle360(magHeading);
				if (_gyroEuler[2] > 4*M_PI || _gyroEuler[2] < -4*M_PI) 
				{
					_gyroEuler[2] = 0;
					printf("Euler[2] out of range dt = %+2.2f gy = %+2.2f gprev = %2.2f", _dt, gz_, gyroPrev[2]);
				}				
			}
	
			
	
			
			
			//Hack to pull back to center like spring in mech DG
			if(accelUpdated_)
			{
				if (abs(ay_) < 0.05f)
				{
					ballcentered = true;
				//	printf("ball centered");
				}
				else
				{
					ballcentered = false;
				}
				if (abs(az_ - 1.0f) < 0.5f)
				{
					
					if (_gyroEuler[1] > 0) _gyroEuler[1] -= 0.05f * _dt;      //move at 0.1 rad / s
					else _gyroEuler[1] += 0.05f * _dt;
				}
				//_Euler[0] = constrainAngle360(_Euler[0]);
				//_Euler[1] = constrainAngle360(_Euler[1]);
			}
			
			if (magyawstill && gyroyawstill && ballcentered)
			{
				wingslevel = true;
				//printf("wings level \r\n");
				if (_Euler[0] > 0) _Euler_Fixed[0] -= 0.1f * _dt;         //move at 0.1 rad / s
					else _Euler_Fixed[0] += 0.1f * _dt;
			}
			else
			{
				wingslevel = false;
			}
			
			_Euler[0] = (1.0*_gyroEuler[0]) + (0.0*_magEuler[0]) + _Euler_Fixed[0];
			_Euler[1] = (1.0*_gyroEuler[1]) + (0.0*_magEuler[1]) + _Euler_Fixed[1];
			_Euler[2] = (1.0*_gyroEuler[2]) + (0.0*_magEuler[2]) + _Euler_Fixed[2];
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
	if (dta <= -M_PI) dta += (M_PI * 2.0);
	return dta;
}

/* Bound angle between 0 and 360 */
float SKFilter::constrainAngle360(float dta) {
	if (dta >=  2*M_PI) dta -= (M_PI * 2.0);
	if (dta < 0) dta += (M_PI * 2.0);
	return dta;
}


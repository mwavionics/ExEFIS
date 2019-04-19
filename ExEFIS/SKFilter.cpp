#include "SKFilter.h"
#include "Vector.h"
#include "math.h"
#include <QDebug>

const int magbuffersize = 50;
const int accelbuffersize = 50;
const int eulerbuffersize = 100;
static float magbuffer[3][magbuffersize];
static float dmagbuffer[3][magbuffersize];
static float accelbuffer[3][accelbuffersize];
static double eulerbuffer[3][eulerbuffersize];
static timeval last;



/**********************************************************************************************//**
* Module: SKFilter::SKFilter
***************************************************************************************************
* @brief	init the SK Filter for AHRS
* @note		
* @todo		
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 11/06/17 - Created & Commented
**************************************************************************************************/
SKFilter::SKFilter()
{
	valid = false;
	
	attitudeOffset = 0;
	
	//Bank, Pitch and Yaw
	_Euler[0] = 0;
	_Euler[1] = 0;
	_Euler[2] = M_PI;
	
	//Reference Coordinates for the chip, not the airplane
	_Euler_Fixed[0] = 0;
	_Euler_Fixed[1] = 0;
	_Euler_Fixed[2] = 0;	
	
	_magEuler[0] = 0;
	_magEuler[1] = 0;
	_magEuler[2] = 0; //Note that magEuler 2 is not on the same plane as _Euler[2]
	
	_gyroEuler[0] = 0;
	_gyroEuler[1] = 0;
	_gyroEuler[2] = 0;
	
	_accelEuler[0] = 0;
	_accelEuler[1] = 0;
	_accelEuler[2] = 0;
	
	gyroPrev[0] = 0;
	gyroPrev[1] = 0;
	gyroPrev[2] = 0;
	
	magPrev[0] = 0;
	magPrev[1] = 0;
	magPrev[2] = 0;
	
	accelPrev[0] = 0;
	accelPrev[1] = 0;
	accelPrev[2] = 0;
	
	magOffset[0] = 0;
	magOffset[1] = 0;
	magOffset[2] = 0;
	
	gyroHeading = 0;
	_initCounter = 0;
	_initialized = false;
	
	acState.wingslevel = 0;
	acState.pitchlevel = 0;
	acState.pitchstill = 0;
	acState.turning = 0;
	acState.ballcentered = 0;
	acState.oneG = 0;
	
	magbufferindex = 0;
}

/**********************************************************************************************//**
* Module: SKFilter::~SKFilter
***************************************************************************************************
* @brief	Deconstructor for the SK Filter AHRS
* @note		
* @todo		
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
* @todo		
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
* @todo			
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 11/06/17 - Created & Commented
*		SSK - 01/31/19 - Updated to use stillaxis and distinct states to reorient
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
			accelbuffer[0][accelbufferindex] = ax;
			accelbuffer[1][accelbufferindex] = ay;
			accelbuffer[2][accelbufferindex] = az;
			accelbufferindex++;
			if (accelbufferindex > accelbuffersize) accelbufferindex = 0;
			ax_ = 0;
			ay_ = 0;
			az_ = 0;
			
			
			for (int i = 0; i < accelbuffersize; i++)
			{			
				
				ax_ += accelbuffer[0][i];
				ay_ += accelbuffer[1][i];
				az_ += accelbuffer[2][i];
			}
			ax_ /= accelbuffersize;
			ay_ /= accelbuffersize;
			az_ /= accelbuffersize;
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
			if (_initCounter > 500) _initialized = true;		
		}	
	
		if (_initialized)
		{	
			
			if (magUpdated_)
			{

				float mag_norm = sqrt((mx_*mx_) + (my_*my_) + (mz_*mz_));
				float xmag = mx_ / mag_norm;
				float ymag = -my_ / mag_norm;
				float zmag = -mz_ / mag_norm;
				
				//Adjust for bank angle and pitch
				float Roll = atan2(zmag, ymag);
				float Pitch = atan2(-zmag, xmag);
				float Yaw = atan2(ymag, xmag);
				
				if (acState.wingslevel >= 0.9f && acState.pitchlevel >= 0.9f)
				{
					magOffset[0] = Roll;
					magOffset[1] = Pitch;
					magOffset[2] = Yaw;
				}
				
				_magEuler[0] = Roll - magOffset[0];
				//_magEuler[0] = constrainAngle360(_magEuler[0]);
				_magEuler[1] = Pitch - magOffset[1]; // - magOffset[1];
				//_magEuler[1] = constrainAngle360(_magEuler[1]);								
				
				X_h = xmag*cos(_Euler[1]) + ymag*sin(-_Euler[0])*sin(_Euler[1]) + zmag*cos(-_Euler[0])*sin(_Euler[1]);
				Y_h = ymag*cos(-_Euler[0]) - zmag*sin(-_Euler[0]);
				hdg = 0.0f;
				hdg = atan2(Y_h, X_h);
//				if(X_h < 0) hdg = M_PI - atang;
//				if (X_h > 0 && Y_h < 0) hdg = -atan2(Y_h, X_h);
//				if (X_h > 0 && Y_h > 0) hdg = (2*M_PI) - atan2(Y_h, X_h);
//				if (X_h == 0 && Y_h < 0) hdg = M_PI / 4;
//				if (X_h == 0 && Y_h > 0) hdg = 3* M_PI / 4;
					
				
				_magEuler[2] = constrainAngle360((2*M_PI)-hdg) ;
				
				//float txmag = xmag*cos(_Euler[1]) + zmag*sin(_Euler[1]);
				//float tymag = ymag*cos(_Euler[0]) + ymag*sin(_Euler[0]);
				//_magEuler[2] = constrainAngle360(atan2(tymag, txmag));
		
			
				//  Need to correct these for bank angle
			
					float steer =  0;//constrainAngle360(yawmag);
				//_magEuler[2] = constrainAngle360(yawmag);
				int i = 1;
				bool found = false;
				for (i = 1; i < 13 && !found; i++)
				{
					if (steer < steerTable[i]) found = true;
				}
				
				float ratio = (steer - steerTable[i - 1]) / (steerTable[i] - steerTable[i - 1]);
			//	_magEuler[2] = (abs(forSteerTable[i] - forSteerTable[i - 1])* ratio) + forSteerTable[i - 1];
			//	_magEuler[2] = constrainAngle360(yawmag);//_magEuler[2]);
								
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
				
				//Set the axis still variables for initialization state
				axisstill.magroll = 100;// - (abs(dmag[0])*AXISSTILLMAGSCALAR);
				axisstill.magpitch = 100;// - (abs(dmag[1])*AXISSTILLMAGSCALAR);
				axisstill.magyaw = 100;// - (abs(dmag[2])*AXISSTILLMAGSCALAR);
				if (axisstill.magroll < 0)axisstill.magroll = 0;
				if (axisstill.magpitch < 0)axisstill.magpitch = 0;
				if (axisstill.magyaw < 0)axisstill.magyaw = 0;				
				
				magPrev[0] = _magEuler[0];
				magPrev[1] = _magEuler[1];
				magPrev[2] = _magEuler[2];
			}
			//Check if the gyro has been updated
			if (gyroUpdated_) 
			{				
				float gxx = gx_;
				float gyy = gy_;
				float gzz = gz_;
					
				/* Calc Integral */
				imu::Vector<3> integral;
				integral[0] = ((gyroPrev[0] * _dt) + ((gxx - gyroPrev[0]) * 0.5f * _dt));
				integral[1] = ((gyroPrev[1] * _dt) + ((gyy - gyroPrev[1]) * 0.5f * _dt));
				integral[2] = ((gyroPrev[2] * _dt) + ((gzz - gyroPrev[2]) * 0.5f * _dt));
				
				//Set the axis still flags
				axisstill.gyroroll = 100 - (abs(integral[0])*AXISSTILLGYROSCALAR);
				axisstill.gyropitch = 100 - (abs(integral[1])*AXISSTILLGYROSCALAR);
				axisstill.gyroyaw = 100 - (abs(integral[2])*AXISSTILLGYROSCALAR);
				if (axisstill.gyroroll < 0) axisstill.gyroroll = 0;
				if (axisstill.gyropitch < 0) axisstill.gyropitch = 0;
				if (axisstill.gyroyaw < 0) axisstill.gyroyaw = 0;
				
				//First Calc Roll Angle
				_gyroEuler[0] = _Euler[0] + integral[0];
				_gyroEuler[0] = constrainAngle180(_gyroEuler[0]);
				
				if (_gyroEuler[0] > 4*M_PI || _gyroEuler[0] < -4*M_PI) 
				{
					_gyroEuler[0] = 0;
					printf("Euler[0] out of range dt = %2.2f gy = %2.2f gprev = %2.2f", _dt, gx_, gyroPrev[0]);
				}
				
				//Now, apply the pitch component and yaw component correctly to previous samples
				//pitch if you're banked, same for yaw - we use the previous because it has more inputs
				//and includes mag and accel...
				_gyroEuler[1] = _Euler[1] + (integral[1] * cos(_Euler[0])) + (integral[2] * sin(_Euler[0]));
				_gyroEuler[1] = constrainAngle180(_gyroEuler[1]);
				
				if (_gyroEuler[1] > 4*M_PI || _gyroEuler[1] < -4*M_PI) 
				{
					_gyroEuler[1] = 0;
					printf("Euler[1] out of range dt = %+2.2f gy = %+2.2f gprev = %2.2f", _dt, gy_, gyroPrev[1]);
				}
				
				float gyroheadingdiff = (integral[2] * cos(_Euler[0])) + (integral[1] * sin(_Euler[0]));	
				gyroHeading = _Euler[2] + gyroheadingdiff; 				
				
			//	if (abs(gyroheadingdiff) < 0.4f) gyroyawstill = true;
			//	else gyroyawstill = false;
								
				_gyroEuler[2] = constrainAngle360(gyroHeading);
				if (_gyroEuler[2] > 4*M_PI || _gyroEuler[2] < -4*M_PI) 
				{
					_gyroEuler[2] = 0;
					printf("Euler[2] out of range dt = %+2.2f gy = %+2.2f gprev = %2.2f", _dt, gz_, gyroPrev[2]);
				}				
			}		
			
			//Hack to pull back to center like spring in mech DG
			//Figure out if you're in steady state here by using airspeed
			//That will say if the airplane is accelerating, if not accelerating,
			//then we can assume pitch angle from accelerometer... 
			float pitchmag = -1.0f;
			float rollmag = -1.0f;
			float totalmag = -1.0f;
			if(accelUpdated_)
			{
				_accelEuler[0] = atan(-ay_ / az_);
				_accelEuler[1] = atan(-ax_ / az_);				
				_accelEuler[2] = atan(-ay_ / ax_);
				
				pitchmag = sqrt((ax_*ax_) + (az_*az_));
				rollmag = sqrt((ay_*ay_) + (az_*az_));
				totalmag = sqrt((ax_*ax_) + (az_*az_) + (ay_*ay_));
				
				axisstill.accelroll = 100 - (abs(_accelEuler[0] - accelPrev[0]) * AXISSTILLACCELSCALAR); 
				axisstill.accelpitch = 100 - (abs(_accelEuler[1] - accelPrev[1]) * AXISSTILLACCELSCALAR); 
				axisstill.accelyaw = 100 - (abs(_accelEuler[2] - accelPrev[2]) * AXISSTILLACCELSCALAR); 
				if (axisstill.accelroll < 0)axisstill.accelroll = 0;
				if (axisstill.accelpitch < 0)axisstill.accelpitch = 0;
				if (axisstill.accelyaw < 0)axisstill.accelyaw = 0;
				
				//_Euler[0] = constrainAngle360(_Euler[0]);
				//_Euler[1] = constrainAngle360(_Euler[1]);
				
				accelPrev[0] = _accelEuler[0];
				accelPrev[1] = _accelEuler[1];
				//accelPrev[2] = _accelEuler[2];
			}
			
			acState.ballcentered = 1.0f - abs(ay_);// < 0.07f ? true : false;
			acState.pitchstill = (axisstill.accelpitch * 0.0033f) + (axisstill.magpitch * 0.0033f) + (axisstill.gyropitch * 0.0033f);// ? true : false;
			acState.turning = 1.0f - ((axisstill.gyroyaw * 0.005) + (axisstill.magyaw*0.005));
			acState.wingslevel = (-acState.turning) + acState.ballcentered;			
			acState.oneG = 1.0f - (abs(1.0f - totalmag));
			acState.pitchlevel = (acState.oneG * 0.25) + (acState.pitchstill*  0.25) + (.25f - abs(_accelEuler[1] - attitudeOffset)) + (.25 - (.25f*acState.turning));
			
			
			if (acState.pitchlevel)
			{					
				if (_Euler[1] > 0)_Euler_Fixed[1] -= 0.05f * _dt;        //move at 0.1 rad / s
				else _Euler_Fixed[1] += 0.05f * _dt;
			}
			
			float fg[3] = { 1.0f, 1.0f, 0.0f };
			float fm[3] = { 0.0f, 0.0f, 1.0f };
			float fa[3] = { 0.0f, 0.0f, 0.0f };

			fg[1] = 1.0f - acState.pitchlevel;
			fa[1] = acState.pitchlevel / 2.0f;
			fm[1] = acState.pitchlevel / 2.0f;
			
			fg[0] = 1.0f - (((1.0f - acState.turning) * 0.33f) + (acState.oneG * 0.33f) + (acState.wingslevel*0.33f));
			fa[0] = (((1.0f - acState.turning) * 0.33f) + (acState.oneG * 0.33f) + (acState.wingslevel * 0.33f));

			fg[2] = 1.0 - ((acState.pitchlevel) * 0.5 + (acState.wingslevel * 0.5));
			fm[2] = ((acState.pitchlevel) * 0.5 + (acState.wingslevel * 0.5));
			
			//Don't need the "spring" with the fixed Euler with the current setup
		
			_Euler[0] = (fg[0]*_gyroEuler[0]) + (fm[0]*_magEuler[0]) + (fa[0]*_accelEuler[0]); //+ _Euler_Fixed[0];
			_Euler[1] = (fg[1]*_gyroEuler[1]) + (fm[1]*_magEuler[1]) + (fa[1]*_accelEuler[1]); //+ _Euler_Fixed[1];
			_Euler[2] = (fg[2]*_gyroEuler[2]) + (fm[2]*_magEuler[2]) + (fa[2]*_accelEuler[2]);// + _Euler_Fixed[2];
						
		}
	}	
}

/* Returns the full 3dof vector*/
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



void SKFilter::setAttitudeOffset(float offset_rads)
{
	attitudeOffset = offset_rads;
}

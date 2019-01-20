#pragma once
#include "Quaternion.h"
#include "Vector.h"
#include "PiTransfer.h"
#include "MPU9250.h"

struct MPU_CAL
{
	float magx;
	float magy;
	float magz;
	float accx;
	float accy;
	float accz;
	float gyrx;
	float gyry;
	float gyrz;	
};

struct INFO_9250
{
	int id;
};


class mpudriver
{
public:
	
	static void imuInterruptHander(int gpio, int level, uint32_t tick);
	
	
	mpudriver();
	mpudriver(float* ppGyroBias, float* ppAccelBias, float* ppMagBias, float* ppMagScale);
	~mpudriver();
	int Init(bool doSelfTest, bool doCalibration, bool doMagCalibration);
	int GetAccelStatus(void);
	int GetMagStatus(void);
	int GetGyrStatus(void);
	int Get9250Info(INFO_9250* info);
	int SetCalibration(MPU_CAL* cal);
	int GetCalibration(MPU_CAL* cal);
	imu::Vector<3> GetEuler(int* status);
	imu::Vector<3> GetAccelerometer(int*status);
	float GetYAccelFiltered(int* status);
	float getRoll(void);
	float getPitch(void);
	float getHeading(void);
	bool getWingsLevel(void);	
	/* Returns the roll angle, rad */
//	float getRoll_rad() {
//		return filter.getRoll_rad();
//	}
//
//	/* Returns the pitch angle, rad */
//	float getPitch_rad() {
//		return filter.getRoll_rad();
//	}
//
//	/* Returns the yaw angle, rad */
//	float getYaw_rad() {
//		return getYaw_rad() ;
//	}
//
//	/* Returns the heading angle, rad */
//	float getHeading_rad() {
//		return filter.getHeading_rad();
//	}
	void resetAlgorithm (void);
	
	static void RunFilter(void);
		
private:
	PiI2C *mpu;
	PiI2C *mag;
	

	

	
};




#pragma once
#include "Quaternion.h"
#include "Vector.h"
#include "PiTransfer.h"
#include "MPU9250.h"
#include "SKFilter.h"

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
	mpudriver(float* ppGyroBias, float* ppAccelBias, float* ppMagBias, float* ppMagScale, float* ppAxisRemap);
	~mpudriver();
	int Init(bool doSelfTest, bool doCalibration, bool doMagCalibration, bool _showmagvectors);
	
	void setSteerCard(int* steerto);
	
	imu::Vector<3> GetEuler(int* status);
	imu::Vector<3> GetAccelerometer(int*status);
	
	float GetYAccelFiltered(int* status);
	float getRoll(void);
	float getPitch(void);
	float getHeading(void);
	
	AC_STATE getAcState(void);
	
	void resetAlgorithm (void);
	
	static void RunFilter(void);
		
private:
	PiI2C *mpu;
	PiI2C *mag;
	
	

	

	
};




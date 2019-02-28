#include <unistd.h>
#include <pigpio.h>
#include <QThread>
#include <math.h>
#include <sys/time.h>

#include "mpudriver.h"
#include "Vector.h"
#include "PiTransfer.h"
#include "MPU9250.h"
#include "quaternionFilters.h"
#include "SKFilter.h"


#define delay(x) usleep(x*1000)

/*
Cal Gyro Raw Data[0] : 5 
Cal Gyro Raw Data[1] : 64 
Cal Gyro Raw Data[2] : 255 
Cal Gyro Raw Data[3] : 248 
Cal Gyro Raw Data[4] : 237 
Cal Gyro Raw Data[5] : 0 
Cal Accel Raw Data[0] : 241 
Cal Accel Raw Data[1] : 115 
Cal Accel Raw Data[2] : 234 
Cal Accel Raw Data[3] : 225 
Cal Accel Raw Data[4] : 39 
Cal Accel Raw Data[5] : 140  *
 **/

// Device constants
static const Gscale_t GSCALE    = GFS_250DPS;
static const Ascale_t ASCALE    = AFS_2G;
static const Mscale_t MSCALE    = MFS_16BITS;
static const Mmode_t  MMODE     = M_100Hz;
static const uint8_t SAMPLE_RATE_DIVISOR = 0x04;    

#define buffersize 100

// scale resolutions per LSB for the sensors
static float aRes, gRes, mRes;
static int quatIndex = 0;

static imu::Vector<3> euler;
static float ax, ay, az, gx, gy, gz, mx, my, mz;

static float quatW[buffersize];
static float quatX[buffersize];
static float quatY[buffersize];
static float quatZ[buffersize];

static float pitchbuffer[buffersize];
static float rollbuffer[buffersize];
static float yawbuffer[buffersize];

static float yaccelbuffer[buffersize];

static struct timeval start;


//MPU9250Master *dev;
MPU9250Passthru *dev;

static SKFilter filter;

// Pin definitions
static const uint8_t intPin = 27;//0;  //GPIO 17;     //  MPU9250 interrupt

// Interrupt support 
static bool runFilter; 

int sensorID;
static float* pGyroBias; 
static float* pAccelBias; 
static float* pMagBias; 
static float* pMagScale;
static float* pAxisRemap;

int GyroRaw[3] = { 
	//1344,
	50000,	
	65528,
	60672
};

int AccelRaw[3] = { 
	61624,
	60129,
	10124
};

// Factory mag calibration and mag bias
static float   magCalibration[3]; 

// Bias corrections for gyro and accelerometer. These can be measured once and
// entered here or can be calculated each time the device is powered on.
static float gyroBias[3], accelBias[3] = { 0, 0, 0 }, 
	magBias[3] = { 36.899, 205.580, -365.1833 }, magScale[3] = { 1, 1, 1 },
    axisremap[3] = { 2.0, 1.0, 0.0 }; 

//static float gyroBias[3] = {, accelBias[3], magBias[3] = { 0, 0, 0 }, magScale[3] = { 1, 1, 1 }; 

mpudriver::mpudriver()
{
	gettimeofday(&start, NULL);
	ax = ay = az = gx = gy = gz = mx = my = mz = 0;
	runFilter = false;
	mpu = new PiI2C(MPU9250::MPU9250_ADDRESS);
	mag = new PiI2C(MPU9250::AK8963_ADDRESS);
	//dev = new MPU9250Master(mpu); 
	dev = new MPU9250Passthru(mpu, mag);	
	
	pGyroBias = gyroBias; 
	pAccelBias = accelBias; 
	pMagBias = magBias; 
	pMagScale = magScale;
	pAxisRemap = axisremap;
	
	filter = SKFilter();
	
	printf("Created MPU9250 9-axis motion sensor...\n");	
}

mpudriver::mpudriver(float* ppGyroBias, float* ppAccelBias, float* ppMagBias, float* ppMagScale, float* ppAxisRemap)
	: mpudriver()
{	
	pGyroBias = ppGyroBias; 
	pAccelBias = ppAccelBias; 
	pMagBias = ppMagBias; 
	pMagScale = ppMagScale;
	pAxisRemap = ppAxisRemap;
}

int mpudriver::Init(bool doSelfTest, bool doCalibration, bool doMagCalibration)
{
	int status = 0;	
	
	mpu->begin();
	mag->begin();
	
	dev->resetMPU9250();

	sensorID = dev->getMPU9250ID();
	
	qDebug("MPU9250  I AM 0x%02X  I should be 0x71 or 0x73\n", sensorID);
	// WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 
	delay(1000);	
	
	//73 for Dev Board, 71 for production
	if(sensorID == 0x71 || sensorID == 0x73) {
    
		qDebug("MPU9250 is online...\n");

		dev->resetMPU9250();      // start by resetting MPU9250

		if(doSelfTest)
		{
		
			float SelfTest[6];         // holds results of gyro and accelerometer self test

			dev->SelfTest(SelfTest);       // Start by performing self test and reporting values

			printf("x-axis self test: acceleration trim within : %+3.3f%% of factory value\n", SelfTest[0]); 
			printf("y-axis self test: acceleration trim within : %+3.3f%% of factory value\n", SelfTest[1]); 
			printf("z-axis self test: acceleration trim within : %+3.3f%% of factory value\n", SelfTest[2]); 
			printf("x-axis self test: gyration trim within : %+3.3f%% of factory value\n", SelfTest[3]); 
			printf("y-axis self test: gyration trim within : %+3.3f%% of factory value\n", SelfTest[4]); 
			printf("z-axis self test: gyration trim within : %+3.3f%% of factory value\n", SelfTest[5]); 
			delay(1000);
		}

		// get sensor resolutions, only need to do this once
		aRes = dev->getAres(ASCALE);
		gRes = dev->getGres(GSCALE);
		mRes = dev->getMres(MSCALE);
		
		//dev->SendCalibrationData(GyroRaw, AccelRaw);
		
		if (doCalibration)
		{		
			// Comment out if using pre-measured, pre-stored offset accel/gyro biases
			dev->calibrateMPU9250(gyroBias, accelBias);      // Calibrate gyro and accelerometers, load biases in bias registers
			printf("accel biases (mg)\n");
			printf("%f\n", 1000.*accelBias[0]);
			printf("%f\n", 1000.*accelBias[1]);
			printf("%f\n", 1000.*accelBias[2]);
			printf("gyro biases (dps)\n");
			printf("%f\n", gyroBias[0]);
			printf("%f\n", gyroBias[1]);
			printf("%f\n", gyroBias[2]);
			
			pAccelBias = accelBias;
			pGyroBias = gyroBias;
			
			usleep(1000*1000); 
		}
		dev->initMPU9250(ASCALE, GSCALE, SAMPLE_RATE_DIVISOR); 
		printf("MPU9250 initialized for active data mode....\n"); 

		//delay(2000);
		QThread::msleep(2000);
		// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
		uint8_t d = dev->getAK8963CID();      // Read WHO_AM_I register for AK8963
		
		
		printf("AK8963  I AM 0x%02x  I should be 0x48\n", d);
		QThread::msleep(500);
		
		
		// Get magnetometer calibration from AK8963 ROM
		dev->initAK8963(MSCALE, MMODE, magCalibration);
		printf("AK8963 initialized for active data mode....\n"); 
		if (doMagCalibration)
		{			
			// Comment out if using pre-measured, pre-stored offset magnetometer biases
			printf("Mag Calibration: Wave device in a figure eight until done!\n");
			delay(4000);
			dev->magcalMPU9250(magBias, magScale);
			printf("Mag Calibration done!\n");
			printf("AK8963 mag biases (mG)\n");
			printf("%f\n", magBias[0]);
			printf("%f\n", magBias[1]);
			printf("%f\n", magBias[2]); 
			printf("AK8963 mag scale (mG)\n");
			printf("%f\n", magScale[0]);
			printf("%f\n", magScale[1]);
			printf("%f\n", magScale[2]); 
			delay(2000);     // add delay to see results before serial spew of data
			printf("Calibration values:\n");
			printf("X-Axis sensitivity adjustment value %+2.2f\n", magCalibration[0]);
			printf("Y-Axis sensitivity adjustment value %+2.2f\n", magCalibration[1]);
			printf("Z-Axis sensitivity adjustment value %+2.2f\n", magCalibration[2]);
			pMagScale = magScale;
			pMagBias = magBias;
		}
		
		
		gpioSetMode(intPin, PI_INPUT);
		gpioSetISRFunc(intPin, RISING_EDGE, 200, mpudriver::imuInterruptHander);
	}
	
	else {

		printf("Could not connect to MPU9250: 0x%02x", sensorID);
		status = -1;
	}	
	delay(1000);
	return status;
}


mpudriver::~mpudriver()
{
}


int mpudriver::GetAccelStatus(void)
{
	return 1;
}


int mpudriver::GetMagStatus(void)
{
	return 1;
}


int mpudriver::GetGyrStatus(void)
{
	return 1;
}


int mpudriver::Get9250Info(INFO_9250* info)
{
	return 1;
}


int mpudriver::SetCalibration(MPU_CAL* cal)
{
	return 1;
}


int mpudriver::GetCalibration(MPU_CAL* cal)
{
	return 1;
}


imu::Vector<3> mpudriver::GetEuler(int* status)
{
	*status = 0;
	float qw = 0, qx = 0, qy = 0, qz = 0;
	for (int i = 0; i < buffersize; i++)
	{
		qw += quatW[i];
		qx += quatX[i];
		qy += quatY[i];
		qz += quatZ[i];
	}
	qw /= buffersize;
	qx /= buffersize;
	qy /= buffersize;
	qz /= buffersize;
	
	return imu::Quaternion(qw, qx, qy, qz).toEuler();
}

float mpudriver::getRoll(void)
{
	float ret = 0.0f;
	for (int i = 0; i < buffersize; i++)
	{
		ret  += rollbuffer[i];
	}
	ret /= buffersize;
	return ret;
}

float mpudriver::getPitch(void)
{
	float ret = 0.0f;
	for (int i = 0; i < buffersize; i++)
	{
		ret  += pitchbuffer[i];
	}
	ret /= buffersize;
	return ret;
}

float mpudriver::getHeading(void)
{
	return filter.getHeading_rad();
}

imu::Vector<3> mpudriver::GetAccelerometer(int*status)
{
	*status = 0;
	return (imu::Vector<3>((double)ax, (double)ay, (double)az));
}

float mpudriver::GetYAccelFiltered(int* status)
{
	*status = 0;
	float ret = 0.0f;
	for (int i = 0; i < buffersize; i++)
	{
		ret  += yaccelbuffer[i];
	}
	ret /= buffersize;
	return ret;
}


static bool inInt = false;

void mpudriver::imuInterruptHander(int gpio, int level, uint32_t tick)
{
	if (!inInt)
	{
		inInt = true;
		RunFilter();
		inInt = false;
	}
	
}



void mpudriver::RunFilter(void)
{
	static int16_t MPU9250Data[7];     // used to read all 14 bytes at once from the MPU9250 accel/gyro
	static int16_t magCount[3];         // Stores the 16-bit signed magnetometer sensor output
	static float ab[3] = { -0.002, 0, 0 };
	
	if (true)
	{	
		runFilter = false;
		if (dev->checkNewAccelGyroData()) {
			// data ready interrupt is detected

		   dev->readMPU9250Data(MPU9250Data);      // INT cleared on any read

		   // Convert the accleration value into g's
		   ax = (float)MPU9250Data[0]*aRes - pAccelBias[0];  
			ay = (float)MPU9250Data[1]*aRes - pAccelBias[1];   
			az = (float)MPU9250Data[2]*aRes- pAccelBias[2];  

			// Convert the gyro value into degrees per second
			gx = (float)MPU9250Data[4]*gRes - pGyroBias[0];  
			gy = (float)MPU9250Data[5]*gRes - pGyroBias[1];  
			gz = (float)MPU9250Data[6]*gRes - pGyroBias[2]; 

			

			
		}
		
		if (dev->checkNewMagData())
		{
			dev->readMagData(magCount);         // Read the x/y/z adc values

			// Calculate the magnetometer values in milliGauss
			// Include factory calibration per data sheet and user environmental corrections
			// Get actual magnetometer value, this depends on scale being set
			mx = (float)magCount[0]*mRes*magCalibration[0] - pMagBias[0];  
			my = (float)magCount[1]*mRes*magCalibration[1] - pMagBias[1];  
			mz = (float)magCount[2]*mRes*magCalibration[2] - pMagBias[2];  
			mx *= pMagScale[0];
			my *= pMagScale[1];
			mz *= pMagScale[2];		
		}
		// Report at 1Hz
		static int usec;
		static timeval last;
		
		timeval tp;
		int v = gettimeofday(&tp, NULL);
		int secs = tp.tv_sec - last.tv_sec;
		deltat = secs * 1000000.0f + (tp.tv_usec - last.tv_usec);
		usec += deltat;
		deltat /= 1000000.0f;
		last = tp;
			
			
		//In the MWA3.5 configuration, Z is roll, Y is pitch, X is yaw
		//That means that the X vector points to gravity
		//The Y Vector points to the wings
		//The Z Vector points to the front of the airplane
		
		//[axis] [g,a,m]
		float imudata[3][3];
		float mappedimudata[3][3];
		imudata[0][0] = gx; imudata[0][1] = ax; imudata[0][2] = mx;
		imudata[1][0] = gy; imudata[1][1] = ay; imudata[1][2] = my;
		imudata[2][0] = gz; imudata[2][1] = az; imudata[2][2] = mz;			
		
		int remap0 = (int)pAxisRemap[0];
		int remap1 = (int)pAxisRemap[1];
		int remap2 = (int)pAxisRemap[2];			
		
		mappedimudata[remap0][0] = gx; mappedimudata[remap0][1] = ax; mappedimudata[remap0][2] = mx;
		mappedimudata[remap1][0] = gy; mappedimudata[remap1][1] = ay; mappedimudata[remap1][2] = my;
		mappedimudata[remap2][0] = gz; mappedimudata[remap2][1] = az; mappedimudata[remap2][2] = mz;
		
		mappedimudata[0][0] *= pAxisRemap[3]*M_PI / 180.0f; mappedimudata[0][1] *= pAxisRemap[4]; mappedimudata[0][2] *= pAxisRemap[5];
		mappedimudata[1][0] *= pAxisRemap[6]*M_PI / 180.0f; mappedimudata[1][1] *= pAxisRemap[7]; mappedimudata[1][2] *= pAxisRemap[8];
		mappedimudata[2][0] *= pAxisRemap[9]*M_PI / 180.0f; mappedimudata[2][1] *= pAxisRemap[10]; mappedimudata[2][2] *= pAxisRemap[11];
		
		
		if (filter.validate(mappedimudata[0][0], mappedimudata[1][0], mappedimudata[2][0], 
				mappedimudata[0][1], mappedimudata[1][1], mappedimudata[2][1], 
				mappedimudata[0][2], mappedimudata[1][2], mappedimudata[2][2]))
		{
		
			filter.update(mappedimudata[0][0],
				mappedimudata[1][0],
				mappedimudata[2][0], 
				mappedimudata[0][1],
				mappedimudata[1][1],
				mappedimudata[2][1], 
				mappedimudata[0][2],
				mappedimudata[1][2],
				mappedimudata[2][2]);
			//filter.getQuaternion(&quatW[quatIndex], &quatX[quatIndex], &quatY[quatIndex], &quatZ[quatIndex]);		
		}
		
		rollbuffer[quatIndex] = filter.getRoll_rad();		
		pitchbuffer[quatIndex] = filter.getPitch_rad(); 					
		//yawbuffer[quatIndex] = filter.getHeading_rad();
		yaccelbuffer[quatIndex] = ay;
	
		quatIndex++;
		if (quatIndex > buffersize) quatIndex = 0;
		
//		last = current;
//				MadgwickQuaternionUpdate(az,
//					ay,
//					ax,
//					gz*M_PI / 180.0f,
//					gy*M_PI / 180.0f,
//					gx*M_PI / 180.0f,
//					-mz,
//					my,
//					mx);

		
//				MahonyQuaternionUpdate(ax, ay, az, gx*M_PI / 180.0f, gy*M_PI / 180.0f, gz*M_PI / 180.0f, mx, my, -mz);
		
		/* only print once per second*/
		if (usec > 250000) {

			usec = 0;
			
			float qw, qx, qy, qz;
			
			euler = filter.getEuler();
			printf("Orientation:, %f, %f, %f, Accelerometer:, %d, %d, %d, Gyro: , %+2.2f, %+2.2f, %+2.2f, Mag:, %+3.3f, %+3.3f, %+3.3f, Quad: %d \n", 
				euler.x() * 180.0f / M_PI,
				euler.y() * 180.0f / M_PI,
				euler.z() * 180.0f / M_PI, 
				(int)(1000*ax),
				(int)(1000*ay),
				(int)(1000*az),
				gx,
				gy,
				gz,
				mx, //this is yaw2
				my,
				mz, //this is yaw1
				filter.quad);
//			printf("Wings Level, %d \n", filter.wingslevel);
//			printf("DMAG[2] = , %+2.4f \n", filter.dmag[2]);

		}
	}
}

void mpudriver::resetAlgorithm(void)
{
	ax = ay = az = gx = gy = gz = mx = my = mz = 0;
}

bool mpudriver::getWingsLevel(void)
{
	return filter.wingslevel;
}

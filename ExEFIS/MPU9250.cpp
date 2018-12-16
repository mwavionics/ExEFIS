/*  Implementation code for MPU9250 class library
 *
 *  Copyright 2017 Tlera Corporation
 *  
 *  Created by Kris Winer
 *
 *  Adapted by Simon D. Levy 19 April 2018
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "MPU9250.h"
#include <unistd.h>
#include <math.h>
#include <QDebug>

#define delay(x) usleep(x*1000)

MPU9250::MPU9250(ByteTransfer * bt)
{
    _mpu = bt;
}

uint8_t MPU9250::getMPU9250ID()
{
    return readMPU9250Register(WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
}

float MPU9250::getMres(Mscale_t mscale) {
    switch (mscale)
    {
        // Possible magnetometer scales (and their register bit settings) are:
        // 14 bit resolution (0) and 16 bit resolution (1)
        case MFS_14BITS:
            _mRes = 10.*4912./8190.; // Proper scale to return milliGauss
            return _mRes;
            break;
        case MFS_16BITS:
            _mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
            return _mRes;
            break;
    }

    // For type safety
    return 0.f;
}

float MPU9250::getGres(Gscale_t gscale) {
    switch (gscale)
    {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        case GFS_250DPS:
            _gRes = 250.0/32768.0;
            return _gRes;
            break;
        case GFS_500DPS:
            _gRes = 500.0/32768.0;
            return _gRes;
            break;
        case GFS_1000DPS:
            _gRes = 1000.0/32768.0;
            return _gRes;
            break;
        case GFS_2000DPS:
            _gRes = 2000.0/32768.0;
            return _gRes;
            break;
    }

    // For type safety
    return 0.f;
}

float MPU9250::getAres(Ascale_t ascale) {
    switch (ascale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case AFS_2G:
            _aRes = 2.0f/32768.0f;
            return _aRes;
            break;
        case AFS_4G:
            _aRes = 4.0f/32768.0f;
            return _aRes;
            break;
        case AFS_8G:
            _aRes = 8.0f/32768.0f;
            return _aRes;
            break;
        case AFS_16G:
            _aRes = 16.0f/32768.0f;
            return _aRes;
            break;
    }

    // For type safety
    return 0.f;
}



void MPU9250::accelWakeOnMotion()
{
    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    uint8_t c = readMPU9250Register(ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
    c = c | 0x01;  // Set accelerometer rate to 1 kHz and bandwidth to 184 Hz
    _mpu->writeRegister(ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master 
    _mpu->writeRegister(INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear  
    _mpu->writeRegister(INT_ENABLE, 0x41);   // Enable data ready (bit 0) and wake on motion (bit 6)  interrupt

    // enable wake on motion detection logic (bit 7) and compare current sample to previous sample (bit 6)
    _mpu->writeRegister(MOT_DETECT_CTRL, 0xC0);  

    // set accel threshold for wake up at  mG per LSB, 1 - 255 LSBs == 0 - 1020 mg), pic 0x19 for 25 mg
    _mpu->writeRegister(WOM_THR, 0x19);

    // set sample rate in low power mode
    /* choices are 0 == 0.24 Hz, 1 == 0.49 Hz, 2 == 0.98 Hz, 3 == 1.958 Hz, 4 == 3.91 Hz, 5 == 7.81 Hz
     *             6 == 15.63 Hz, 7 == 31.25 Hz, 8 == 62.50 Hz, 9 = 125 Hz, 10 == 250 Hz, and 11 == 500 Hz
     */
    _mpu->writeRegister(LP_ACCEL_ODR, 0x02);

    c = readMPU9250Register(PWR_MGMT_1);
    _mpu->writeRegister(PWR_MGMT_1, c | 0x20);     // Write bit 5 to enable accel cycling

    gyromagSleep();
    delay(100); // Wait for all registers to reset 

}

void MPU9250::resetMPU9250()
{
    // reset device
    _mpu->writeRegister(PWR_MGMT_1, 0x80); // Set bit 7 to reset MPU9250
    delay(100); // Wait for all registers to reset 
}

void MPU9250::readMPU9250Data(int16_t * destination)
{
    uint8_t rawData[14];  // x/y/z accel register data stored here
    _mpu->readRegisters(ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
    destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;   
    destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;  
    destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;  
    destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ; 
}

void MPU9250::readAccelData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    _mpu->readRegisters(ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void MPU9250::readGyroData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    _mpu->readRegisters(GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

bool MPU9250Passthru::checkNewAccelGyroData()
{
    return (readMPU9250Register(INT_STATUS) & 0x01);
}

bool MPU9250::checkWakeOnMotion()
{
    return (readMPU9250Register(INT_STATUS) & 0x40);
}


int16_t MPU9250::readGyroTempData()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    _mpu->readRegisters(TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
    return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}


void MPU9250::initMPU9250(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor, bool passthru)
{  
    // wake up device
    //_mpu->writeRegister(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
    _mpu->writeRegister(PWR_MGMT_1, 0x80); // Clear sleep mode bit (6), enable all sensors 
    delay(100); // Wait for all registers to reset 

    // get stable time source
    _mpu->writeRegister(PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    delay(200); 

    // ----------------------------------

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    if (passthru) _mpu->writeRegister(CONFIG, 0x03);  

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    _mpu->writeRegister(SMPLRT_DIV, sampleRateDivisor);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
    // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = readMPU9250Register(GYRO_CONFIG); // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    c = c & ~0x02; // Clear Fchoice bits [1:0] 
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | gscale << 3; // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    _mpu->writeRegister(GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = readMPU9250Register(ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | ascale << 3; // Set full scale range for the accelerometer 
    _mpu->writeRegister(ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = readMPU9250Register(ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    _mpu->writeRegister(ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    if (passthru) {
        //_mpu->writeRegister(INT_PIN_CFG, 0x22);    
        _mpu->writeRegister(INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear  
    }

    else {

        // enable master mode
        _mpu->writeRegister(USER_CTRL, I2C_MST_EN);
    }

    _mpu->writeRegister(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    delay(100);
}

void MPU9250::magcalMPU9250(float * dest1, float * dest2) 
{
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

    if(_Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(_Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms

    for(ii = 0; ii < sample_count; ii++) {

        readMagData(mag_temp);  // Read the mag data   

        for (int jj = 0; jj < 3; jj++) {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }

        if(_Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
        if(_Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0]*_mRes*_magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*_mRes*_magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*_mRes*_magCalibration[2];  

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
/*SSK - to do
 *if accel is not within 400mg of zero, don't calibrate the accel
 *need to add 3 calibration routines and take average of the 2 for each axis?
 *this will be hardcoded eventually.
 *Need to read - what causes accel bias?
 **/


/**
 **Send calibration data, this is the raw values that go into the IMU
 ***/
void MPU9250::SendCalibrationData(int *gyro_bias, int *accel_bias)
{
	uint8_t data[12];
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	  data[1] = (-gyro_bias[0] / 4)       & 0xFF;  // Biases are additive, so change sign on calculated average gyro biases
	  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4)       & 0xFF;
	data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4)       & 0xFF;

	// Push gyro biases to hardware registers
	_mpu->writeRegister(XG_OFFSET_H, data[0]);
	_mpu->writeRegister(XG_OFFSET_L, data[1]);
	_mpu->writeRegister(YG_OFFSET_H, data[2]);
	_mpu->writeRegister(YG_OFFSET_L, data[3]);
	_mpu->writeRegister(ZG_OFFSET_H, data[4]);
	_mpu->writeRegister(ZG_OFFSET_L, data[5]);
	
	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = { 0, 0, 0 };  // A place to hold the factory accelerometer trim biases
    _mpu->readRegisters(XA_OFFSET_H, 2, &data[0]);  // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
	_mpu->readRegisters(YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
	_mpu->readRegisters(ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);

	uint32_t mask = 1uL;  // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 };  // Define array to hold mask bit for each accelerometer bias axis

	for(int ii = 0 ; ii < 3 ; ii++) {
		if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01;  // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8);  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	//    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	//    data[1] = (accel_bias_reg[0])      & 0xFF;
	//    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	//    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	//    data[3] = (accel_bias_reg[1])      & 0xFF;
	//    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	//    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	//    data[5] = (accel_bias_reg[2])      & 0xFF;
	//    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	    // Apparently this is not working for the acceleration biases in the MPU-9250
	    // Are we handling the temperature correction bit properly?
	    // Push accelerometer biases to hardware registers
	_mpu->writeRegister(XA_OFFSET_H, data[0]);
	_mpu->writeRegister(XA_OFFSET_L, data[1]);
	_mpu->writeRegister(YA_OFFSET_H, data[2]);
	_mpu->writeRegister(YA_OFFSET_L, data[3]);
	_mpu->writeRegister(ZA_OFFSET_H, data[4]);
	_mpu->writeRegister(ZA_OFFSET_L, data[5]);
}

/*
 *NOTE that the pointers passed in are for real unit viewing, not for actual calibration
 **/
void MPU9250::calibrateMPU9250(float * dest1, float * dest2)
{  
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    _mpu->writeRegister(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
    // else use the internal oscillator, bits 2:0 = 001
    _mpu->writeRegister(PWR_MGMT_1, 0x01);  
    _mpu->writeRegister(PWR_MGMT_2, 0x00);
    delay(200);                                    

    // Configure device for bias calculation
    _mpu->writeRegister(INT_ENABLE, 0x00);   // Disable all interrupts
    _mpu->writeRegister(FIFO_EN, 0x00);      // Disable FIFO
    _mpu->writeRegister(PWR_MGMT_1, 0x00);   // Turn on internal clock source
    _mpu->writeRegister(I2C_MST_CTRL, 0x00); // Disable I2C master
    _mpu->writeRegister(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    _mpu->writeRegister(USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    _mpu->writeRegister(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    _mpu->writeRegister(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    _mpu->writeRegister(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    _mpu->writeRegister(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    _mpu->writeRegister(USER_CTRL, 0x40);   // Enable FIFO  
    _mpu->writeRegister(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    _mpu->writeRegister(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    _mpu->readRegisters(FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
	if (packet_count)
	{
		
	
		for (ii = 0; ii < packet_count; ii++)
		{
			int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
			_mpu->readRegisters(FIFO_R_W, 12, &data[0]);    // read data for averaging
			accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);     // Form signed 16-bit integer for each sample in FIFO
			accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
			accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);    
			gyro_temp[0]  = (int16_t)(((int16_t)data[6] << 8) | data[7]);
			gyro_temp[1]  = (int16_t)(((int16_t)data[8] << 8) | data[9]);
			gyro_temp[2]  = (int16_t)(((int16_t)data[10] << 8) | data[11]);

			accel_bias[0] += (int32_t) accel_temp[0];    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
			accel_bias[1] += (int32_t) accel_temp[1];
			accel_bias[2] += (int32_t) accel_temp[2];
			gyro_bias[0]  += (int32_t) gyro_temp[0];
			gyro_bias[1]  += (int32_t) gyro_temp[1];
			gyro_bias[2]  += (int32_t) gyro_temp[2];

		}
		accel_bias[0] /= (int32_t) packet_count;    // Normalize sums to get average count biases
		accel_bias[1] /= (int32_t) packet_count;
		accel_bias[2] /= (int32_t) packet_count;
		gyro_bias[0]  /= (int32_t) packet_count;
		gyro_bias[1]  /= (int32_t) packet_count;
		gyro_bias[2]  /= (int32_t) packet_count;
	}
	else
	{
		
	}

	//SSK commented out these lines
	//This isn't Z any more, it's X
	//below is original code
   // if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
   // else {accel_bias[2] += (int32_t) accelsensitivity;}
	if (accel_bias[0] > 0L) {accel_bias[0] -= (int32_t) accelsensitivity; }  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[0] += (int32_t) accelsensitivity; }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    _mpu->writeRegister(XG_OFFSET_H, data[0]);
    _mpu->writeRegister(XG_OFFSET_L, data[1]);
    _mpu->writeRegister(YG_OFFSET_H, data[2]);
    _mpu->writeRegister(YG_OFFSET_L, data[3]);
    _mpu->writeRegister(ZG_OFFSET_H, data[4]);
    _mpu->writeRegister(ZG_OFFSET_L, data[5]);
	
	for (int jj = 0; jj < 6; jj+= 2)
	{
		int realval = (int)(data[jj] << 8 & 0x0000FF00) + data[jj + 1];
		printf("Cal Gyro Raw Data[%d] : %d \n", jj, data[jj]); 
		printf("Cal Gyro Raw Data[%d] : %d \n", jj+1, data[jj+1]);
		printf("Cal Gyro Real Data[%d] : %d \n", jj/2, realval); 
	}
	

    // Output scaled gyro biases for display in the main program
    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    _mpu->readRegisters(XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    _mpu->readRegisters(YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    _mpu->readRegisters(ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for(ii = 0; ii < 3; ii++) {
        if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
      _mpu->writeRegister(XA_OFFSET_H, data[0]);
      _mpu->writeRegister(XA_OFFSET_L, data[1]);
      _mpu->writeRegister(YA_OFFSET_H, data[2]);
      _mpu->writeRegister(YA_OFFSET_L, data[3]);
      _mpu->writeRegister(ZA_OFFSET_H, data[4]);
      _mpu->writeRegister(ZA_OFFSET_L, data[5]);
	
	for (int jj = 0; jj < 6; jj += 2)
	{
		int realval = (int)(data[jj] << 8 & 0x0000FF00) + data[jj + 1];
		printf("Cal Accel Raw Data[%d] : %d \n", jj, data[jj]); 
		printf("Cal Accel Raw Data[%d] : %d \n", jj + 1, data[jj + 1]);
		printf("Cal Accel Real Data[%d] : %d \n", jj / 2, realval); 
	}

    // Output scaled accelerometer biases for display in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
    uint8_t selfTest[6];
    int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
    float factoryTrim[6];
    uint8_t FS = 0;

    _mpu->writeRegister(SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
    _mpu->writeRegister(CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    _mpu->writeRegister(GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
    _mpu->writeRegister(ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    _mpu->writeRegister(ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

    for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

        _mpu->readRegisters(ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
        aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

        _mpu->readRegisters(GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
        gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    }

    for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    // Configure the accelerometer for self-test
    _mpu->writeRegister(ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
    _mpu->writeRegister(GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    delay(25);  // Delay a while to let the device stabilize

    for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

        _mpu->readRegisters(ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
        aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

        _mpu->readRegisters(GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
        gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    }

    for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }   

    // Configure the gyro and accelerometer for normal operation
    _mpu->writeRegister(ACCEL_CONFIG, 0x00);  
    _mpu->writeRegister(GYRO_CONFIG,  0x00);  
    delay(25);  // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    selfTest[0] = readMPU9250Register(SELF_TEST_X_ACCEL); // X-axis accel self-test results
    selfTest[1] = readMPU9250Register(SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
    selfTest[2] = readMPU9250Register(SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
    selfTest[3] = readMPU9250Register(SELF_TEST_X_GYRO);  // X-axis gyro self-test results
    selfTest[4] = readMPU9250Register(SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
    selfTest[5] = readMPU9250Register(SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
    factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
    factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
    factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
    factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
    factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get percent, must multiply by 100
    for (int i = 0; i < 3; i++) {
        destination[i]   = 100.0f*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.0f;   // Report percent differences
        destination[i+3] = 100.0f*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.0f; // Report percent differences
    }

}

void MPU9250Passthru::writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    _mag->writeRegister(subAddress, data);
}

void MPU9250Master::writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    uint8_t count = 1;

    _mpu->writeRegister(I2C_SLV0_ADDR, AK8963_ADDRESS); // set slave 0 to the AK8963 and set for write
    _mpu->writeRegister(I2C_SLV0_REG, subAddress); // set the register to the desired AK8963 sub address
    _mpu->writeRegister(I2C_SLV0_DO, data); // store the data for write
    _mpu->writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count); // enable I2C and send 1 byte
}

void MPU9250Passthru::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    _mag->readRegisters(subAddress, count, dest);
}

void MPU9250Master::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    _mpu->writeRegister(I2C_SLV0_ADDR, AK8963_ADDRESS | I2C_READ_FLAG); // set slave 0 to the AK8963 and set for read
    _mpu->writeRegister(I2C_SLV0_REG, subAddress); // set the register to the desired AK8963 sub address
    _mpu->writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count); // enable I2C and request the bytes
    delay(1); // takes some time for these registers to fill
    _mpu->readRegisters(EXT_SENS_DATA_00, count, dest); // read the bytes off the MPU9250 EXT_SENS_DATA registers
}

uint8_t MPU9250::readAK8963Register(uint8_t subAddress)
{
    uint8_t buffer = 0;
    readAK8963Registers(subAddress, 1, &buffer);
    return buffer;
}

bool MPU9250Passthru::checkNewMagData()
{
    return readAK8963Register(AK8963_ST1) & 0x01;
}

bool MPU9250Master::checkNewData(void)
{
    return (readMPU9250Register(INT_STATUS) & 0x01);
}

uint8_t MPU9250::getAK8963CID()
{
    return readAK8963Register(WHO_AM_I_AK8963);
}

void MPU9250::gyromagSleep()
{
    uint8_t temp = 0;
    temp = readAK8963Register(AK8963_CNTL);
    writeAK8963Register(AK8963_CNTL, temp & ~(0x0F) ); // Clear bits 0 - 3 to power down magnetometer  
    temp = readMPU9250Register(PWR_MGMT_1);
    _mpu->writeRegister(PWR_MGMT_1, temp | 0x10);     // Write bit 4 to enable gyro standby
    delay(10); // Wait for all registers to reset 
}

void MPU9250::gyromagWake(Mmode_t mmode)
{
    uint8_t temp = 0;
    temp = readAK8963Register(AK8963_CNTL);
    writeAK8963Register(AK8963_CNTL, temp | mmode ); // Reset normal mode for  magnetometer  
    temp = readMPU9250Register(PWR_MGMT_1);
    _mpu->writeRegister(PWR_MGMT_1, 0x01);   // return gyro and accel normal mode
    delay(10); // Wait for all registers to reset 
}

void MPU9250::readMagData(int16_t * destination)
{
    uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    readAK8963Registers(AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
        destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
        destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
    }
}

void MPU9250::initAK8963(Mscale_t mscale, uint8_t Mmode, float * magCalibration)
{
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    writeAK8963Register(AK8963_CNTL, 0x00); // Power down magnetometer  
    delay(10);
    writeAK8963Register(AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    delay(10);
    readAK8963Registers(AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
    magCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
    magCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
    magCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
    _magCalibration[0] = magCalibration[0];
    _magCalibration[1] = magCalibration[1];
    _magCalibration[2] = magCalibration[2];
    _Mmode = Mmode;
    writeAK8963Register(AK8963_CNTL, 0x00); // Power down magnetometer  
    delay(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    writeAK8963Register(AK8963_CNTL, mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
    delay(10);
}

uint8_t MPU9250::readMPU9250Register(uint8_t subAddress)
{
    uint8_t data = 0;
    _mpu->readRegisters(subAddress, 1, &data);
    return data;
}

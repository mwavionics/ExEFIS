In order to get the BCM2835 library into QT:

use a wget call on the raspberry pi to download the file, then tar and install:

http://www.airspayce.com/mikem/bcm2835/index.html

wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.52.tar.gz
# download the latest version of the library, say bcm2835-1.xx.tar.gz, then:
tar zxvf bcm2835-1.xx.tar.gz
cd bcm2835-1.xx
./configure
make
sudo make check
sudo make install

Then after install, copy the copy the libbcm2835.a from /usr/local/lib to /usr/lib and the bcm2835.h from /usr/local/include to /usr/include and rsync your sysroot again.

Then in your QT Project's *.pro file: Add LIBS += -lbcm2835

Then you can include the bcm2835 in your project.

Remember that bcm2835 requires to run as root, so you may turn root ssh on and update that in you project.

To allow root SSH:
If you want to login as root using SSH or WinSCP you need to edit the config of SSHD, do this:
Login, and edit this file: sudo nano /etc/ssh/sshd_config.
Find this line: PermitRootLogin without-password.
Edit: PermitRootLogin yes.
Close and save file.
reboot or restart sshd service using: /etc/init.d/ssh restart.

Working with the pi-plates DAQC version 1:
SPI can run no faster than 300kHz
I run it with a div1024 at 244khz
The pi plate is slow to responsd to CS1 as well as the GPIO25 pullup

Delays must be in place betwee:
 - GPIO25 going high and CS1 going low (can be simultaineous, or close)
 - Starting the SPI clock and reading or writing data

 There also must be a good delay between write and read when you clock out the data

 11/3/17 - Updateing timing
 For Reading the ADC:
 -> CS1 LOW -> Wait 50us -> Clock the Data -> Wait 100uS -> Release CS -> wait 200uS -> Then read
 When reading, read one byte at at time, sleep 100uS between bytes
 When reading multiple things in a row, wait 1mS before reading the next "thing"

 11/5/17 - updated to use the piplate DAQC2. Needed to change the address offset, as well as get the Caldata for the ADC

 12/01/17 - needed to use the wiringPi library for the interrupt driven rotary encoders 
 Had to Add LIBS += -lwiringPi to the *.pro file
 Follow directions on wiringpi website, then use the make tools to resync the libraries
 Have to comment out delayMicroseconds() function in wiringPi.h to avoid the BCM definition conflict
 TODO: might investigate moving all IO over to the wiringpi library

12/05/17
Still can't get to run on device - crashing with bm2835 library
To run independantly on the device, you copy the executable to the device
make the file an executable with chmod +x command
From there you can run it.
Want to create a script to do that - need to solve the debugging problem first. Can make it happen with XMING at the moment.
Need to make a way to close the program... At least as a debug.

12/09/17
Learning more on BNO055 - it tends to list and drift a bit in the car
I added a debug screen to read the registers, "completely calibrated" is returning false.
To calibrate manually, you do the figure 8 motion to calibrate magnetometer, then lay on each side, still for 2-3 seconds, and you should get ff from the calibration register 0x35

01/15/18
For the Rpi "Large" Orientation, in the horizon_instrument.cpp:
/*Move the Origin to the center so it will rotate*/
	float azpix = height() / 180.0f;
	painter.translate(width() / 2, (height() /2 )+ (azpix * _azimuth));//(height() / 2) + _azimuth));
	painter.rotate(_angle);

And then in BNO055.cpp, the orientation is setup like:
write8(BNO055_AXIS_MAP_CONFIG_ADDR, 0x18);//REMAP_CONFIG_P2); // P0-P7, Default is P1
	                delay(10);
					write8(BNO055_AXIS_MAP_SIGN_ADDR, 0x04);//REMAP_SIGN_P2); // P0-P7, Default is P1
	                delay(10);
**************************************************************************************************************************************

07/10/18
Removed BNO055 and added invensense - new hardware platform 2.0

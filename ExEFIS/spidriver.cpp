#include "spidriver.h"
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <iostream>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <QDebug>



spidriver::spidriver(const char* device)
{
	int mode = 7; //set to 4 for CS Hi
	int speed = 250000;
	int bits = 8;
	
	fd = open(device, O_RDWR);
	if (fd < 1) while (1)
			;	
	int ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
	if (ret == -1)
		printf("can't set spi mode");	
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		printf("can't set bits per word");
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printf("can't set max speed hz");
}


spidriver::~spidriver()
{
}


void spidriver::init(void)
{
	
}


int spidriver::transferdata(unsigned char* data, int length)
{
	//return(read(fd, data, length));
	char tx[4];
	spi_ioc_transfer tr;
	
	tr.tx_buf = (unsigned long)tx;
	tr.rx_buf = (unsigned long)data;
	tr.len = length;
	tr.delay_usecs = 10;
	tr.speed_hz = 250000;
	tr.bits_per_word = 0;	


	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret == -1)
		qDebug("can't send spi message");
	return (1);
}

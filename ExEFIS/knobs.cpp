#include "knobs.h"
#include "RotaryEncoder.h"

/* CAN socket stuff*/
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <string.h>
#include <stdio.h>

knobs::knobs()
{	
	//NOTE: These are switched for 3.5 version as configured.
	//left = new RotaryEncoder(12, 13, 19, 0);
	right = new RotaryEncoder(5, 6, 22, 1);
#if 0
	int s;
	struct sockaddr_can addr;
	struct ifreq ifr;

	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	strcpy(ifr.ifr_name, "can0");
	ioctl(s, SIOCGIFINDEX, &ifr);

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	bind(s, (struct sockaddr *)&addr, sizeof(addr));
	
	struct can_frame frame;
	frame.can_dlc = 8;
	frame.can_id = 0x0D0;
	frame.data[0] = 0xFF;
	frame.data[1] = 0xAA;
	ssize_t nbytes = write(s, &frame, sizeof(struct can_frame));
	
	/* Receive some CAN*/
	

	nbytes = read(s, &frame, sizeof(struct can_frame));

	if (nbytes < 0) {
		printf("can raw socket read");
		//return 1;
	}

	/* paranoid check ... */
	if (nbytes < sizeof(struct can_frame)) {
		fprintf(stderr, "read: incomplete CAN frame\n");
		//return 1;
	}

	/* do something with the received CAN frame */
#endif	
	
}


knobs::~knobs()
{
}

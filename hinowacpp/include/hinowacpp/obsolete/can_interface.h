#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

//#include <inttypes.h>
//#include <string.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <unistd.h>
//#include <poll.h>
//#include <sys/types.h>
//#include <sys/socket.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include "ros/ros.h"
#include <linux/can.h>
#include <linux/can/raw.h>

#define SOCKETCAN_ERROR   -1
#define SOCKETCAN_TIMEOUT -2
#define BUFFER_SIZE 1

namespace hinowacpp
{
	class CAN
	{
		public:
			CAN(int, bool);
			virtual ~CAN();
			int readData(uint16_t address, struct can_frame &frame);
			int writeData(uint16_t address, int data);
			void filter(uint16_t filter);

		private:
			int _virt;
			int _CANbus;
			int openCAN();
			int CANsocket;
			struct sockaddr_can addr;
			struct ifreq ifr;
			struct can_frame frame;

			//uint8_t* dec2hex(int dec); future implementation?
			//int hex2dec(uint8_t hex);			
	};
}

typedef struct {
	uint16_t id;
	uint8_t dlc;
	uint8_t data[8];
} my_can_frame;

typedef struct {
	uint32_t size; //!< in bytes (1=unit8_t, 2=uint16_t, 3=unit24_t, 4=uint32_t)
	uint32_t data;
} Socketcan_t;

#endif

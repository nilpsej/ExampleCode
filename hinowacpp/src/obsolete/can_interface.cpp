/*
#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <cstdlib>
#include <cmath>
#include <linux/can.h>
#include <linux/can/raw.h>
*/
#include <hinowacpp/can_interface.h>

namespace hinowacpp
{
	CAN::CAN(int bus, bool virt) {
		_CANbus = bus;
		_virt = virt;
		openCAN();
	}

	CAN::~CAN() {
		close(CANsocket);
	}

	int CAN::readData(uint16_t address, struct can_frame &frame) {
/*
		socklen_t len = sizeof(addr);
		int nbytes;
		nbytes = recvfrom(CANsocket, &frame, sizeof(struct can_frame),
        	0, (struct sockaddr*)&addr, &len);

		if (nbytes < sizeof(struct can_frame)) {
			ROS_ERROR("ReadData: Incomplete CAN frame read.");
			return -1;
		}

    		// get interface name of the received CAN frame
		ifr.ifr_ifindex = addr.can_ifindex;
		ioctl(CANsocket, SIOCGIFNAME, &ifr);
		/*
		for(int i=0;i<4;i++) {
			printf("%02X", frame.data[i]);
		}
		printf("\n");
		
		//printf("ADDRESS REQUESTED: %X.\n", address);
		//printf("CAN address: %d, frame ID: %d.\n", address, frame.can_id);
		return 1;
*/
	}

	int CAN::writeData(uint16_t address, int data) {
/*				
		uint8_t hexData[8];
		int quotient;
		int num = abs(data);
		int nbytes;	
		//socklen_t len = sizeof(addr);	
		struct can_frame send_frame;
		send_frame.can_id = address;
		send_frame.can_dlc = 8;
		
		if(data < 0) hexData[0] = 1;
		else hexData[0] = 0;

		for(int i=1;i<8;i++) {
			//printf("%X", send_frame.data[i]);
			hexData[i] = num % 16;
			quotient = num/16;	
			num = quotient;
		}

		for(int i=0;i<8;i++) {
			send_frame.data[i] = hexData[i];
		}
		
		
		nbytes = sendto(CANsocket, &send_frame, sizeof(struct can_frame),
		0, (struct sockaddr*)&addr, sizeof(addr));
		/*while(nbytes < 0){
			nbytes = sendto(CANsocket, &send_frame, sizeof(struct can_frame),
			0, (struct sockaddr*)&addr, sizeof(addr));
		}
	
		
		if(nbytes < 0) {
		// Error, no data written
		printf("Nbytes is less than 0: %i, failed to send CAN data.\n",nbytes);
		throw std::runtime_error("socketcan: Could not write data to CAN-bus\n");
		}	

		//printf("|CAN frame written| ID: %X, Data: %d.\n", address, data);
*/
		return 1;
	
	}

	int CAN::openCAN() {
		std::string CANbus = "can";
		if(_virt == true){	
			CANbus = "vcan";
		}
		CANbus += std::to_string(_CANbus);
		char const* CANbus_char = CANbus.c_str();

		if((CANsocket = socket(PF_CAN, SOCK_RAW, CAN_RAW))<0){
		ROS_ERROR("Error while opening socket");
		throw std::runtime_error("CAN socket error while opening. Exiting for safety.");
		return -1;
		}
		//setup code (dont modify)
		strcpy(ifr.ifr_name, CANbus_char);
		ioctl(CANsocket, SIOCGIFINDEX, &ifr);
	
		addr.can_family  = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;

		printf("[CAN INIT] %s opened at index %d\n", ifr.ifr_name, ifr.ifr_ifindex);

		if(bind(CANsocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		ROS_ERROR("Error in socket bind");
		throw std::runtime_error("CAN error while binding. Exiting for safety.");
		return -1;	
		}
		if(_virt == true){	
			//set to non blocking			
			fcntl(CANsocket, F_SETFL, O_NONBLOCK);
		}



	}

	void CAN::filter(uint16_t filter){
/*
		//filter the address that will be read, stops encoder n from returning encoder m value.
		struct can_filter rfilter[1];
		rfilter[0].can_id   = filter;
		rfilter[0].can_mask = CAN_SFF_MASK;
		//rfilter[0].can_mask = 0x0FFFFFFF;
		//rfilter[0].can_mask = CAN_SFF_MASK;

		int result = setsockopt(CANsocket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
		//printf("Filtering result: %d.\n", result);
*/
	}
}



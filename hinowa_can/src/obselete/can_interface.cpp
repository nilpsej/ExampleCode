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
#include <hinowa_can/can_interface.h>
#include <std_msgs/String.h>

#include <sstream>
#include <linux/types.h>
#include <linux/socket.h>


namespace hinowa_can
{
	CAN::CAN(int bus, bool virt, ros::NodeHandle nh){
		_CANbus = bus;
		_virt = virt;
		_nh = nh;
		openCAN();

		for(int i=0; i<8; i++){
			RX_pairs[i].id = RX_ids[i];
		}

		_can_publisher = nh.advertise<hinowa_can_msgs::frameArray>("/hinowa/CAN_RX", 1000);
		_can_subscriber = _nh.subscribe<hinowa_can_msgs::valve_control>("/hinowa/CAN_TX", 1000, &CAN::valveControlCallback, this);
	}

	CAN::~CAN(){
		//close(CANsocket);
		//printf("~reached.\n");
	}

	void CAN::valveControlCallback(const hinowa_can_msgs::valve_control::ConstPtr& instruction){//CAN WRITE, automatically called. 

		//int numTX = sizeof(instruction->ids)/sizeof(instruction->ids[0]); //number of CAN_TX ID/DATA pairs
		for(int i=0; i<15; i++){
//WRITE TO CAN CODE		
			uint8_t hexData[8];
			int quotient;
			int num = abs(instruction->data[i]);
			int nbytes;	
			//socklen_t len = sizeof(addr);	
			struct can_frame send_frame;
			send_frame.can_id = instruction->ids[i];
			send_frame.can_dlc = 8;
		
			if(instruction->data[i] < 0) hexData[0] = 1;
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
		
			nbytes = write(CANsocket, &send_frame, sizeof(struct can_frame));

//Data write check
			if(nbytes < 0) {
			// Error: no data written, exit for safety.
				printf("Nbytes is less than 0: %i, failed to send CAN data.\n",nbytes);
				throw std::runtime_error("hinowa_can/can_interface.cpp: (Socketcan) Could not write data to CAN-bus.\n");
			}
//---------------//
		}
		//printf("|CAN frame written| ID: %X, Data: %d.\n", address, data);
	}

	int CAN::readData(){ //CANsocket read function.
		struct can_frame frame;
		for(int i=0; i<8;i++){
			rfilter[0].can_id   = RX_ids[i];
			rfilter[0].can_mask = CAN_SFF_MASK;

			setsockopt(CANsocket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

			nbytes = read(CANsocket, &frame, sizeof(struct can_frame));

			if (nbytes < sizeof(struct can_frame)) {
				//ROS_ERROR("ReadData: Incomplete CAN frame read.\n");
				throw std::runtime_error("ReadData: Incomplete CAN frame read.\n");
				return -1;
			}
	    		// get interface name of the received CAN frame
	//		ifr.ifr_ifindex = addr.can_ifindex;
	//		ioctl(CANsocket, SIOCGIFNAME, &ifr);
	/*
			for(int i=0;i<4;i++) {
				printf("%02X", frame.data[i]);
			}
			printf("\n");
	*/
			//printf("Frame address: %X.\n", frame.can_id);
			assignData(frame);
		}
		publish();
		ros::spinOnce();
		return 1;

	}

	void CAN::assignData(struct can_frame frame){ //Assign data thats been read from the CAN to frames that are meaningful to main loop.
//new
		if(frame.can_id == 0x098) printf("X updated.\n");
		else if(frame.can_id == 0x099) printf("Y updated.\n");
		else printf("Read address: %X.\n", frame.can_id);
		for(int i=0; i<8; i++){
			if(RX_pairs[i].id == frame.can_id) memcpy(RX_pairs[i].data, frame.data, 8);//RX_pair[i].data = frame.data;
		}		
	
	}

	int CAN::openCAN(){
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

/*
		//filter the address that will be read, stops encoder n from returning encoder m value.
		rfilter[0].can_id   = 0x090;
		rfilter[0].can_mask = CAN_SFF_MASK;
		rfilter[1].can_id   = 0x098;
		rfilter[1].can_mask = CAN_SFF_MASK;
		rfilter[2].can_id   = 0x099;
		rfilter[2].can_mask = CAN_SFF_MASK;
		rfilter[3].can_id   = 0x181;
		rfilter[3].can_mask = CAN_SFF_MASK;
		rfilter[4].can_id   = 0x182;
		rfilter[4].can_mask = CAN_SFF_MASK;
		rfilter[5].can_id   = 0x183;
		rfilter[5].can_mask = CAN_SFF_MASK;
		rfilter[6].can_id   = 0x184;
		rfilter[6].can_mask = CAN_SFF_MASK;
		rfilter[7].can_id   = 0x185;
		rfilter[7].can_mask = CAN_SFF_MASK;

		int result = setsockopt(CANsocket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
*/		
/*
		if(_virt == true){	
			//set to non blocking			
			fcntl(CANsocket, F_SETFL, O_NONBLOCK);
		}
*/
	}

	void CAN::publish(){ //READ DATA, publisher reads data from the CAN and pushes it to the main loop.
		hinowa_can_msgs::frame RX_frame;
		hinowa_can_msgs::frameArray RX_array;
		RX_frame.header.stamp = ros::Time::now();

		for(int i=0; i<8; i++){
			RX_frame.id = RX_pairs[i].id;
			memcpy(&RX_frame.data, &RX_pairs[i].data, 8);
			RX_array.frames.push_back(RX_frame);
		}
/*
		for(int i=0;i<4;i++) {
			printf("%02X", RX_frame.data[i]);
		}
		printf("\n");
*/
		_can_publisher.publish(RX_array);
	}

}



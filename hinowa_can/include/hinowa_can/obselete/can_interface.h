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
#include <bits/stdc++.h> 
#include <hinowa_can_msgs/valve_control.h>
#include <hinowa_can_msgs/frame.h>
#include <hinowa_can_msgs/frameArray.h>
#include <hinowa_can/RX_struct.h>

namespace hinowa_can
{
	class CAN
	{
		public:
			CAN(int bus, bool virt, ros::NodeHandle nh);
			virtual ~CAN();
			void valveControlCallback(const hinowa_can_msgs::valve_control::ConstPtr& instruction);
			void assignData(struct can_frame frame);
			int readData();
			void publish();

		private:
			int _virt;
			int _CANbus;
			ros::NodeHandle _nh;
			int openCAN();
			int CANsocket;
			int nbytes;
			RX_pair_t RX_pairs [8];
			struct sockaddr_can addr;
			socklen_t len = sizeof(addr);
			struct ifreq ifr;
			struct can_filter rfilter[1];

			uint16_t RX_ids[8] = {0x090, 0x098, 0x099, 0x181, 0x182, 0x183, 0x184, 0x185};

			ros::Publisher _can_publisher;		
			ros::Subscriber _can_subscriber;	
	};
}

#endif

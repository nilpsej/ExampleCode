#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

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
#include <hinowa_can/TX_struct.h>

namespace hinowa_can
{
	class can_raw
	{
			static const hinowa_can_msgs::valve_control::ConstPtr& instruction;
		public:
			can_raw(int bus, bool virt, ros::NodeHandle nh);
			virtual ~can_raw();
			//void subscribe();
			void valveControlCallback(const hinowa_can_msgs::valve_control::ConstPtr& instruction);

		private:
			int _virt;
			int _CANbus;
			ros::NodeHandle _nh;
			int openCAN();
			int CANsocket;
			struct sockaddr_can addr;
			socklen_t len = sizeof(addr);
			struct ifreq ifr;
			TX_pair_t prev_TX_pairs [15];
			//Removed Ids: 50 (motor start)
					//	paint, STB1, STB2,   LT,    RT,   STB3,  STB4   ,           VALVES               , TOP/BOT
			uint16_t TX_ids[15] = {0x060, 0x091, 0x092, 0x093, 0x094, 0x095, 0x096, 0x102, 0x103, 0x104, 0x105, 0x106, 0x198, 0x199, 0x200};
		
			ros::Subscriber _can_sub;	
	};
}

#endif

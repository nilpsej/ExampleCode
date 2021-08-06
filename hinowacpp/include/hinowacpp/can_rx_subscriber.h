#ifndef HINOWACPP__CAN_RX_SUBSCRIBER_H
#define HINOWACPP__CAN_RX_SUBSCRIBER_H

//#include <sstream>
//#include <hinowacpp/can_interface.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Float64MultiArray.h>
//#include "std_msgs/String.h"
//#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/ros.h>
#include <bits/stdc++.h> 
#include <hinowa_can_msgs/frame.h>
#include <hinowa_can_msgs/frameArray.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <hinowacpp/RX_structs.h>

namespace hinowacpp
{
	class CAN_RX_SUBSCRIBER
	{
		private:
			ros::NodeHandle _nh;
			ros::Subscriber _sub;
			RX_pair_t RX_pairs[7];
			uint16_t RX_ids[7] = {0x89, 0x090, 0x181, 0x182, 0x183, 0x184, 0x185};
			int n = 7;

		public:
			//RX_frames RX_data;
			//hinowa_can_msgs::frame rx_frame;
			CAN_RX_SUBSCRIBER();
			CAN_RX_SUBSCRIBER(ros::NodeHandle nh);
			~CAN_RX_SUBSCRIBER();
			void storeData(const hinowa_can_msgs::frameArray::ConstPtr& rx_frames);
			void subscribe();
			RX_pair_t getData(uint16_t ID);
	};
}

#endif

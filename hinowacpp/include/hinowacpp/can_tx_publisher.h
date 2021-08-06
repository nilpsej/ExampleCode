#ifndef HINOWACPP__CAN_TX_PUBLISHER_H
#define HINOWACPP__CAN_TX_PUBLISHER_H

//#include <sstream>
//#include <hinowacpp/can_interface.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Float64MultiArray.h>
//#include "std_msgs/String.h"
//#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/ros.h>
#include <bits/stdc++.h> 
//#include <hinowa_can_msgs/frame.h>
//#include <hinowa_can_msgs/frameArray.h>
#include <hinowa_can_msgs/valve_control.h>
#include <hinowacpp/TX_structs.h>


namespace hinowacpp
{
	class CAN_TX_PUBLISHER
	{
		private:
			ros::NodeHandle _nh;
			ros::Publisher _pub;
			//a similar process is necessary in hinowa.cpp, update both to refer to a yaml file that initialises addresses for these objects. Will need to include base & valve control addresses in yaml, not just joint addresses. 
			TX_pair_t TX_pairs [15];
			uint16_t TX_ids[15] = {0x060, 0x091, 0x092, 0x093, 0x094, 0x095, 0x096, 0x102, 0x103, 0x104, 0x105, 0x106, 0x198, 0x199, 0x200};


		public:
			CAN_TX_PUBLISHER();
			CAN_TX_PUBLISHER(ros::NodeHandle nh);
			~CAN_TX_PUBLISHER();
			void addData(uint16_t ID, int data);
			void publish();
	};
}

#endif

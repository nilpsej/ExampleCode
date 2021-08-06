#ifndef HINOWACPP__JOINT_SUBSCRIBER_H
#define HINOWACPP__JOINT_SUBSCRIBER_H

#include <sstream>
#include <hinowacpp/can_interface.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <bits/stdc++.h> 

namespace hinowacpp
{
	class JOINT_SUBSCRIBER
	{
		private:
			ros::NodeHandle _nh;
			ros::Subscriber _sub;
			float _J6Position = 3.14159;
			float _J8Position = 1.57071;
			float _position = 0;

		public:
			JOINT_SUBSCRIBER();
			JOINT_SUBSCRIBER(ros::NodeHandle nh);
			~JOINT_SUBSCRIBER();
			void storeData(const sensor_msgs::JointState::ConstPtr& joint_states);
			double subscribe(std::string joint_name);
	};
}

#endif

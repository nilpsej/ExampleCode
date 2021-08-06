#ifndef HINOWA_JOINT_ERROR_SUBSCRIBER_H
#define HINOWA_JOINT_ERROR_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <bits/stdc++.h> 
#include <control_msgs/JointTrajectoryControllerState.h>
#include <hinowa_joint_errors/joint_struct.h>

namespace hinowa_joint_errors
{
	class JOINT_ERROR_SUBSCRIBER
	{
		private:
			ros::NodeHandle _nh;
			ros::Subscriber _sub;
			int _prevSOT = 0;
			

		public:
			JOINT_ERROR_SUBSCRIBER();
			JOINT_ERROR_SUBSCRIBER(ros::NodeHandle nh);
			~JOINT_ERROR_SUBSCRIBER();
			void init(std::vector<std::string> joint_names);
			void storeData(const control_msgs::JointTrajectoryControllerState::ConstPtr& joint_states);
			void subscribe(ros::Duration dt, int SOT, int control_state);
			float joint_position_errors [5] = {};
			std::vector<Joint> joint_errors;
	};
}

#endif

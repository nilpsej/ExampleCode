#ifndef HINOWA_JOINT_ERROR_PUBLISHER_H
#define HINOWA_JOINT_ERROR_PUBLISHER_H

#include <ros/ros.h>
#include <bits/stdc++.h> 
#include <hinowa_joint_errors/PID_terms.h>
#include <hinowa_joint_errors/joint_struct.h>

namespace hinowa_joint_errors
{
	class JOINT_ERROR_PUBLISHER
	{
		private:
			ros::NodeHandle _nh;
			ros::Publisher _pub;

		public:
			JOINT_ERROR_PUBLISHER();
			JOINT_ERROR_PUBLISHER(ros::NodeHandle nh);
			~JOINT_ERROR_PUBLISHER();
			void addErrorTerms(std::vector<Joint> joint_errors_, std::vector<float> actual_accelerations_, std::vector<double> joint_commands_);
			void publish();
			std::vector<Joint> joint_errors;
			std::vector<float> actual_accelerations;
			std::vector<float> acceleration_errors;
			std::vector<double> joint_commands;
			
	};
}

#endif

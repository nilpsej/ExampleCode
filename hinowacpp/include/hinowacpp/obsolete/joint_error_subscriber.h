#ifndef HINOWACPP__JOINT_ERROR_SUBSCRIBER_H
#define HINOWACPP__JOINT_ERROR_SUBSCRIBER_H

//#include <sstream>
//#include <hinowacpp/can_interface.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Float64MultiArray.h>
//#include "std_msgs/String.h"
//#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <bits/stdc++.h> 
#include <control_msgs/JointTrajectoryControllerState.h>

namespace hinowacpp
{
	class JOINT_ERROR_SUBSCRIBER
	{
		private:
			ros::NodeHandle _nh;

			ros::Subscriber _sub;
			float _jointErrors [5][5] = {};
			//float _velocityErrors [5][5] = {};
			int _jointNum [5] = {1, 4, 6, 7, 8};
			/* for reference of matrix indexes.
			float _J1PError = 0; float _J1IError = 0; float _J1DError = 0; float _J1PErrorPrev = 0; float _J1AccumulativeError = 0;
			float _J4PError = 0; float _J4IError = 0; float _J4DError = 0; float _J4PErrorPrev = 0; float _J4AccumulativeError = 0;
			float _J6PError = 0; float _J6IError = 0; float _J6DError = 0; float _J6PErrorPrev = 0; float _J6AccumulativeError = 0;
			float _J7PError = 0; float _J7IError = 0; float _J7DError = 0; float _J7PErrorPrev = 0; float _J7AccumulativeError = 0;
			float _J8PError = 0; float _J8IError = 0; float _J8DError = 0; float _J8PErrorPrev = 0; float _J8AccumulativeError = 0;
			*/
			int _prevSOT = 0;
			

		public:
			JOINT_ERROR_SUBSCRIBER();
			JOINT_ERROR_SUBSCRIBER(ros::NodeHandle nh);
			~JOINT_ERROR_SUBSCRIBER();
			void storeData(const control_msgs::JointTrajectoryControllerState::ConstPtr& joint_states);
			void subscribe(ros::Duration dt, int SOT, int control_state);
			float joint_position_errors [5] = {};
			//void setData(int data);
	};
}

#endif

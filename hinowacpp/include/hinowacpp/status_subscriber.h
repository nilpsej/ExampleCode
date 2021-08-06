#ifndef HINOWACPP__STATUS_SUBSCRIBER_H
#define HINOWACPP__STATUS_SUBSCRIBER_H

//#include <sstream>
//#include "std_msgs/String.h"
//#include <bits/stdc++.h> 
//#include <sensor_msgs/JointState.h>
//#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <ros/ros.h>

namespace hinowacpp
{
	class STATUS_SUBSCRIBER
	{
		private:
			ros::NodeHandle _nh;
			ros::Subscriber _sub;
			actionlib_msgs::GoalStatus _trajectory_status;

		public:
			STATUS_SUBSCRIBER();
			STATUS_SUBSCRIBER(ros::NodeHandle nh);
			~STATUS_SUBSCRIBER();
			void storeData(const actionlib_msgs::GoalStatusArray::ConstPtr& goal_status);
			uint8_t subscribe();
	};
}

#endif

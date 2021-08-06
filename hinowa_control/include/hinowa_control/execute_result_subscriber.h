#ifndef HINOWACPP__EXECUTE_RESULT_SUBSCRIBER_H
#define HINOWACPP__EXECUTE_RESULT_SUBSCRIBER_H

//#include <sstream>
#include <ros/ros.h>
//#include <bits/stdc++.h>
//#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
//#include <actionlib_msgs/GoalStatus.h>
//#include <moveit_msgs/ExecuteTrajectoryResult.h>

namespace control
{
	class EXECUTE_RESULT_SUBSCRIBER
	{
		private:
			ros::NodeHandle _nh;
			ros::Subscriber _sub;
			int _result = 0;
			int _status = 0;
			int _prevResult = 0;
			int _prevStatus = 0;
			actionlib_msgs::GoalID _ID;
			actionlib_msgs::GoalID _prevID;
			int _newResult = 0;

		public:
			EXECUTE_RESULT_SUBSCRIBER();
			EXECUTE_RESULT_SUBSCRIBER(ros::NodeHandle nh);
			~EXECUTE_RESULT_SUBSCRIBER();
			void storeData(const moveit_msgs::ExecuteTrajectoryActionResult::ConstPtr& execute_result);
			std::tuple<int, int> subscribe(); //uint8_t
	};
}

#endif

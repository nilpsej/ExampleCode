#ifndef HINOWACPP__PLAN_RESULT_SUBSCRIBER_H
#define HINOWACPP__PLAN_RESULT_SUBSCRIBER_H

//#include <sstream>
#include <ros/ros.h>
//#include <bits/stdc++.h>
#include <moveit_msgs/MoveGroupActionResult.h>
//#include <actionlib_msgs/GoalStatus.h>
//#include <moveit_msgs/MoveGroupResult.h>


namespace control
{
	class PLAN_RESULT_SUBSCRIBER
	{
		private:
			ros::NodeHandle _nh;
			ros::Subscriber _sub;
			int _result = 1;
			int _status = 0;
			int _prevResult = 0;

		public:
			PLAN_RESULT_SUBSCRIBER();
			PLAN_RESULT_SUBSCRIBER(ros::NodeHandle nh);
			~PLAN_RESULT_SUBSCRIBER();
			void storeData(const moveit_msgs::MoveGroupActionResult::ConstPtr& plan_result);
			int subscribe(); //uint8_t
	};
}

#endif

//#include <stdlib.h>
//#include <math.h>
//#include <stdexcept>
//#include <ros/ros.h>
//#include<bits/stdc++.h>
#include <hinowacpp/status_subscriber.h>

namespace hinowacpp
{

	STATUS_SUBSCRIBER::STATUS_SUBSCRIBER()
	{

	}

	STATUS_SUBSCRIBER::STATUS_SUBSCRIBER(ros::NodeHandle nh)
	{
		this->_nh = nh;
		
	}

	STATUS_SUBSCRIBER::~STATUS_SUBSCRIBER()
	{

	}
	
	void STATUS_SUBSCRIBER::storeData(const actionlib_msgs::GoalStatusArray::ConstPtr& goal_status)
	{
		if(!goal_status->status_list.empty()){
			_trajectory_status = goal_status->status_list.back();
		}
	}


	uint8_t STATUS_SUBSCRIBER::subscribe()	
	{
		
		this->_sub = _nh.subscribe<actionlib_msgs::GoalStatusArray>("/hinowa/joint_trajectory_controller/follow_joint_trajectory/status", 1000, &STATUS_SUBSCRIBER::storeData, this);
		//printf("Status output: %d.\n", _trajectory_status.status);
		return _trajectory_status.status;
		
	}



}

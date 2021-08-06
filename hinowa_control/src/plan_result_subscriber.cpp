#include <ros/ros.h>
#include <hinowa_control/plan_result_subscriber.h>
//#include <stdlib.h>
//#include <math.h>
//#include <stdexcept>
//#include <bits/stdc++.h>

namespace control
{

	PLAN_RESULT_SUBSCRIBER::PLAN_RESULT_SUBSCRIBER()
	{

	}

	PLAN_RESULT_SUBSCRIBER::PLAN_RESULT_SUBSCRIBER(ros::NodeHandle nh)
	{
		this->_nh = nh;
		
	}

	PLAN_RESULT_SUBSCRIBER::~PLAN_RESULT_SUBSCRIBER()
	{

	}
	
	void PLAN_RESULT_SUBSCRIBER::storeData(const moveit_msgs::MoveGroupActionResult::ConstPtr& plan_result)
	{
		actionlib_msgs::GoalStatus status = plan_result->status;
		moveit_msgs::MoveGroupResult result = plan_result->result;
		this->_status = status.status; //should be a int value for the result of planning
		this->_result = result.error_code.val;
/*
		if(!goal_status->status_list.empty()){
			_trajectory_status = goal_status->status_list.back();
		}
*/
	}

	
	int PLAN_RESULT_SUBSCRIBER::subscribe()	
	{	
		this->_sub = _nh.subscribe<moveit_msgs::MoveGroupActionResult>("/move_group/result", 1000, &PLAN_RESULT_SUBSCRIBER::storeData, this);
		if(_result != _prevResult){
			//printf("Plan - Status: %d, Result: %d. \n", _status, _result);
		}
		_prevResult = _result;
		
		return _result;
	}



}

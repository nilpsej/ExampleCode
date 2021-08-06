#include <hinowa_control/execute_result_subscriber.h>
//#include <stdlib.h>
//#include <math.h>
//#include <stdexcept>
//#include <ros/ros.h>
//#include <bits/stdc++.h>

namespace control
{

	EXECUTE_RESULT_SUBSCRIBER::EXECUTE_RESULT_SUBSCRIBER()
	{

	}

	EXECUTE_RESULT_SUBSCRIBER::EXECUTE_RESULT_SUBSCRIBER(ros::NodeHandle nh)
	{
		this->_nh = nh;
		
	}

	EXECUTE_RESULT_SUBSCRIBER::~EXECUTE_RESULT_SUBSCRIBER()
	{

	}
	
	void EXECUTE_RESULT_SUBSCRIBER::storeData(const moveit_msgs::ExecuteTrajectoryActionResult::ConstPtr& execute_result)
	{
		actionlib_msgs::GoalStatus status = execute_result->status;
		moveit_msgs::ExecuteTrajectoryResult result = execute_result->result;
		this->_status = status.status;
		this->_ID = status.goal_id;
		this->_result = result.error_code.val;
		//this->_result = execute_result->val; //should be an int value for the result of executing.
/*
		if(!goal_status->status_list.empty()){
			_trajectory_status = goal_status->status_list.back();
		}
*/
	}


	std::tuple<int, int> EXECUTE_RESULT_SUBSCRIBER::subscribe()	
	{	
		int executionSuccess = -2;
		this->_sub = _nh.subscribe<moveit_msgs::ExecuteTrajectoryActionResult>("/execute_trajectory/result", 1000, &EXECUTE_RESULT_SUBSCRIBER::storeData, this);
		if((_ID.id != _prevID.id) || (_ID.stamp.toSec() != _prevID.stamp.toSec())){
			_newResult = 1;
			printf("Execute - Status: %d, Result: %d. \n", _status, _result);
			if(_result == 1 && _status == 3) executionSuccess = 1;
			else executionSuccess = 0;
		}
		else _newResult = 0;
		_prevResult = _result;
		_prevStatus = _status;
		_prevID = this->_ID;

		std::tuple <int, int> output(executionSuccess, _newResult);	
		return output;
	}



}

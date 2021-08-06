//#include <stdlib.h>
//#include <math.h>
//#include <stdexcept>
//#include <ros/ros.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Float64MultiArray.h>
//#include "std_msgs/String.h"
//#include <geometry_msgs/Twist.h>
//#include<bits/stdc++.h> 
//#include <control_msgs/JointTrajectoryControllerState.h>
//#include <trajectory_msgs/JointTrajectoryPoint.h>
//#include <iostream>
//#include <fstream>
#include <hinowacpp/joint_error_subscriber.h>
#include <sensor_msgs/JointState.h>

namespace hinowacpp
{

	JOINT_ERROR_SUBSCRIBER::JOINT_ERROR_SUBSCRIBER()
	{
	}

	JOINT_ERROR_SUBSCRIBER::JOINT_ERROR_SUBSCRIBER(ros::NodeHandle nh)
	{
		this->_nh = nh;
	}

	JOINT_ERROR_SUBSCRIBER::~JOINT_ERROR_SUBSCRIBER()
	{
	}

	void JOINT_ERROR_SUBSCRIBER::storeData(const control_msgs::JointTrajectoryControllerState::ConstPtr& joint_states)
	{
		trajectory_msgs::JointTrajectoryPoint desired = joint_states->desired;
		trajectory_msgs::JointTrajectoryPoint actual = joint_states->actual;
		//_jointNames = joint_states->name;
		//const trajectory_msgs::JointTrajectoryPoint point = joint_states->desired;
		_jointErrors[0][0] = desired.positions[0]-actual.positions[0];
		_jointErrors[1][0] = desired.positions[1]-actual.positions[1];
		_jointErrors[2][0] = desired.positions[2]-actual.positions[2];
		_jointErrors[3][0] = desired.positions[3]-actual.positions[3];
		_jointErrors[4][0] = desired.positions[4]-actual.positions[4];
/*		
		_velocityErrors[0][0] = desired.velocities[0]-actual.velocities[0];
		_velocityErrors[1][0] = desired.velocities[1]-actual.velocities[1];
		_velocityErrors[2][0] = desired.velocities[2]-actual.velocities[2];
		_velocityErrors[3][0] = desired.velocities[3]-actual.velocities[3];
		_velocityErrors[4][0] = desired.velocities[4]-actual.velocities[4];
*/		
	}


	void JOINT_ERROR_SUBSCRIBER::subscribe(ros::Duration dt, int SOT, int control_state)	
	{
		if(SOT == 1 && _prevSOT != 1){ //if trajectory is being executed on this loop, but wasn't on the previous loop..
			for(int i = 0; i < 5; i++){
				_jointErrors[i][4] = 0; //set accumulative error to 0
				_jointErrors[i][3] = 0; //set differential error to 0
//				_velocityErrors[i][4] = 0; //set accumulative error to 0
//				_velocityErrors[i][3] = 0; //set differential error to 0
			}
		}
		else if(SOT == 1 && _prevSOT == 1){
			this->_sub = _nh.subscribe<control_msgs::JointTrajectoryControllerState>("/hinowa/joint_trajectory_controller/state", 1000, &JOINT_ERROR_SUBSCRIBER::storeData, this);
			std::ofstream outfile("/home/josh/workspace/src/hinowacpp/pid_errors.csv",std::ios::app);			
			for(int i = 0; i < 5; i++){
//POSITION ERROR CALCULATIONS
				_jointErrors[i][4] += _jointErrors[i][0]; //accumulative error += error for current loop
				_jointErrors[i][1] = _jointErrors[i][4]*dt.toSec(); //integral error = accumulative error*dt
				_jointErrors[i][2] = (_jointErrors[i][0]-_jointErrors[i][3])/dt.toSec(); 
				//derivative error = current error - previous error
		
				_jointErrors[i][3] = _jointErrors[i][0]; //update previous error for each joint for next loop
//VELOCITY ERROR CALCULATIONS
/*
				_velocityErrors[i][4] += _velocityErrors[i][0]; //accumulative error += error for current loop
				_velocityErrors[i][1] = _velocityErrors[i][4]*dt.toSec(); //integral error = accumulative error*dt
				_velocityErrors[i][2] = (_velocityErrors[i][0]-_velocityErrors[i][3])/dt.toSec();
				//derivative error = current error - previous error
		
				_velocityErrors[i][3] = _velocityErrors[i][0]; //update previous error for each joint for next loop

				//outfile<<"Joint "<<_jointNum[i]<<","<<_jointErrors[i][0]<<","<<_jointErrors[i][1]<<","<<_jointErrors[i][2]<<","<<"Joint "<<_jointNum[i]<<","<<_velocityErrors[i][0]<<","<<_velocityErrors[i][1]<<","<<_velocityErrors[i][2]<<",";
*/

				outfile<<"Joint "<<_jointNum[i]<<","<<_jointErrors[i][0]<<","<<_jointErrors[i][1]<<","<<_jointErrors[i][2]<<",";
				
      				
			}
			outfile<<"\n";
			outfile.close();

			for(int i = 0; i < 5; i++){
				joint_position_errors[i] = _jointErrors[i][0];
			}

		}
		_prevSOT = SOT;	
/* example code for what the above indexed code should do if not functioning as expected.
//Joint 1
			_J1AccumulativeError += _J1PError; 
			_J1IError = _J1AccumulativeError*dt.toSec();
			_J1DError = (_J1PError-_J1PErrorPrev)/dt.toSec();

		_J1PErrorPrev = _J1PError;

		printf("JOINT 1 ERRORS - P: %lf, I: %lf, D: %lf.\n", _J1PError, _J1IError, _J1DError);
*/
			
	}



}

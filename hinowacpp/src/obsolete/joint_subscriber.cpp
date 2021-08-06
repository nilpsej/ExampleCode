//#include <stdlib.h>
//#include <math.h>
//#include <stdexcept>
//#include <ros/ros.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Float64MultiArray.h>
//#include "std_msgs/String.h"
//#include<bits/stdc++.h> 

#include <hinowacpp/joint_subscriber.h>
#include <hinowacpp/can_interface.h>
#include <sensor_msgs/JointState.h>

namespace hinowacpp
{

	JOINT_SUBSCRIBER::JOINT_SUBSCRIBER()
	{

	}

	JOINT_SUBSCRIBER::JOINT_SUBSCRIBER(ros::NodeHandle nh)
	{
		this->_nh = nh;
		//this->_valveId = valveId;
		//this-> _enabled = enabled;
		//this->_data = 0;
		//this->_pub = _nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
		
	}

	JOINT_SUBSCRIBER::~JOINT_SUBSCRIBER()
	{

	}
	
	void JOINT_SUBSCRIBER::storeData(const sensor_msgs::JointState::ConstPtr& joint_states)
	{
		//_jointNames = joint_states->name;
		_J6Position = joint_states->position[2];
		_J8Position = joint_states->position[4];
		
	}


	double JOINT_SUBSCRIBER::subscribe(std::string joint_name)	
	{
		
		this->_sub = _nh.subscribe<sensor_msgs::JointState>("/hinowa/joint_states", 1000, &JOINT_SUBSCRIBER::storeData, this);
		if(joint_name == "joint_end_bottom") this->_position = 3.14159-(_J6Position);
		else this->_position = _J8Position;
		//std::cout<<"J9: "<<_position<<"\n";
		return double(_position);
		
	}



}

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
#include <hinowacpp/can_tx_publisher.h>

namespace hinowacpp
{

	CAN_TX_PUBLISHER::CAN_TX_PUBLISHER()
	{
	}

	CAN_TX_PUBLISHER::CAN_TX_PUBLISHER(ros::NodeHandle nh)
	{
		this->_nh = nh;
		this->_pub = nh.advertise<hinowa_can_msgs::valve_control>("/hinowa/CAN_TX", 1000); 
		for(int i=0; i<15; i++){
			TX_pairs[i].id = TX_ids[i];
		}
	}

	CAN_TX_PUBLISHER::~CAN_TX_PUBLISHER()
	{
		
	}

	void CAN_TX_PUBLISHER::addData(uint16_t ID, int data)
	{
		for(int i=0; i<15; i++){
			if(TX_pairs[i].id == ID) TX_pairs[i].data = data;
		}
	}

	void CAN_TX_PUBLISHER::publish()
	{
		hinowa_can_msgs::valve_control actuation_data;
		actuation_data.header.stamp = ros::Time::now();
		for(int i=0; i<15; i++){
			actuation_data.ids.push_back(TX_pairs[i].id);
			actuation_data.data.push_back(TX_pairs[i].data);
		}
		_pub.publish(actuation_data);
		ros::spinOnce();
	}
}

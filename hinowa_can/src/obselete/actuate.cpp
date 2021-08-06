#include <hinowa_can/actuate.h>
#include <std_msgs/String.h>

#include <sstream>
#include <linux/types.h>
#include <linux/socket.h>

namespace hinowa_can
{
	actuate::actuate(ros::NodeHandle nh){
		//_nh = nh;
		_actuate_publisher = nh.advertise<hinowa_can_msgs::valve_control>("/hinowa/actuation_data", 1000);
	}

	actuate::~actuate(){
		//printf("~reached.\n");
	}

	void actuate::publish(){ //publish fake actuation data
		hinowa_can_msgs::valve_control actuation_data;
		uint16_t ID = 0x105;
		double data = 10000;
		actuation_data.ids.push_back(ID);
		actuation_data.data.push_back(data);

		_actuate_publisher.publish(actuation_data);
/*
		//typedef __u8[8] uint8_t[8];
		hinowa_can_msgs::frame RX_frame;
		hinowa_can_msgs::frameArray RX_array;

		RX_frame.header.stamp = ros::Time::now();
		RX_frame.id = joint_1.can_id;
		for (int i = 0; i < 8; i++){
   			 RX_frame.data[i] = joint_1.data[i];
   		}
		RX_array.frames.push_back(RX_frame);

		RX_frame.header.stamp = ros::Time::now();
		RX_frame.id = joint_4.can_id;
		for (int i = 0; i < 8; i++){
   			 RX_frame.data[i] = joint_4.data[i];
   		}
		RX_array.frames.push_back(RX_frame);

		RX_frame.header.stamp = ros::Time::now();
		RX_frame.id = joint_6.can_id;
		for (int i = 0; i < 8; i++){
   			 RX_frame.data[i] = joint_6.data[i];
   		}
		RX_array.frames.push_back(RX_frame);

		RX_frame.header.stamp = ros::Time::now();
		RX_frame.id = joint_7.can_id;
		for (int i = 0; i < 8; i++){
   			 RX_frame.data[i] = joint_7.data[i];
   		}
		RX_array.frames.push_back(RX_frame);

		RX_frame.header.stamp = ros::Time::now();
		RX_frame.id = joint_8.can_id;
		for (int i = 0; i < 8; i++){
   			 RX_frame.data[i] = joint_8.data[i];
   		}
		RX_array.frames.push_back(RX_frame);



		_can_publisher.publish(RX_array);
*/
	}
}



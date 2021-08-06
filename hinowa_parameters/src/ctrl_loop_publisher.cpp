#include <hinowa_parameters/ctrl_loop_publisher.h>

namespace hinowa_parameters
{

	ctrl_loop_publisher::ctrl_loop_publisher()
	{
	}

	ctrl_loop_publisher::ctrl_loop_publisher(ros::NodeHandle nh)
	{
		this->_nh = nh;
		this->_pub = nh.advertise<hinowa_parameters::loop_control>("/hinowa/loop2hwi", 1000); 

	}

	ctrl_loop_publisher::~ctrl_loop_publisher()
	{
		
	}

	void ctrl_loop_publisher::addData(int state, int target_aruco_ID, std::string active_valve_block, int ball_valve, int reset_levelling)
	{
		this->state = state;
		this->active_valve_block = active_valve_block; 
		this->ball_valve = ball_valve; 
		this->reset_levelling = reset_levelling;
		this->target_aruco_ID = target_aruco_ID;


	}

	void ctrl_loop_publisher::publish()
	{
		hinowa_parameters::loop_control msg;
		msg.header.stamp = ros::Time::now();
		msg.state = state;
		msg.target_aruco_ID = target_aruco_ID;
		msg.active_valve_block = active_valve_block;
		msg.ball_valve = ball_valve;
		msg.reset_levelling = reset_levelling;
		_pub.publish(msg);
		ros::spinOnce();

	}
}

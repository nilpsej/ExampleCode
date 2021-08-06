#include <hinowa_parameters/ctrl_loop_subscriber.h>

namespace hinowa_parameters
{

	ctrl_loop_subscriber::ctrl_loop_subscriber()
	{
	}

	ctrl_loop_subscriber::ctrl_loop_subscriber(ros::NodeHandle nh)
	{
		this->_nh = nh;
	}

	ctrl_loop_subscriber::~ctrl_loop_subscriber()
	{
	}
	
	void ctrl_loop_subscriber::storeData(const hinowa_parameters::loop_control::ConstPtr& params)
	{
		this->state = params->state;
		this->target_aruco_ID = params->target_aruco_ID;
		this->active_valve_block = params->active_valve_block;
		this->ball_valve = params->ball_valve;
		this->reset_levelling = params->reset_levelling;
	}


	void ctrl_loop_subscriber::subscribe()	
	{
		this->_sub = _nh.subscribe<hinowa_parameters::loop_control>("/hinowa/loop2hwi", 1000, &ctrl_loop_subscriber::storeData, this);
	}
}

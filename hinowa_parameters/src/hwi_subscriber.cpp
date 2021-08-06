#include <hinowa_parameters/hwi_subscriber.h>

namespace hinowa_parameters
{

	hwi_subscriber::hwi_subscriber()
	{
	}

	hwi_subscriber::hwi_subscriber(ros::NodeHandle nh)
	{
		this->_nh = nh;
	}

	hwi_subscriber::~hwi_subscriber()
	{
	}
	
	void hwi_subscriber::storeData(const hinowa_parameters::hinowa_hwi::ConstPtr& params)
	{
		this->levelled = params->levelled;
		this->state_of_trajectory = params->state_of_trajectory;
		this->loop_hz = params->loop_hz;
	}


	void hwi_subscriber::subscribe()	
	{
		this->_sub = _nh.subscribe<hinowa_parameters::hinowa_hwi>("/hinowa/hwi2loop", 1000, &hwi_subscriber::storeData, this);
	}
}

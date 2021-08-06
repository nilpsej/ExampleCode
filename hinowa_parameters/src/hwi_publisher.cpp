#include <hinowa_parameters/hwi_publisher.h>

namespace hinowa_parameters
{

	hwi_publisher::hwi_publisher()
	{
	}

	hwi_publisher::hwi_publisher(ros::NodeHandle nh)
	{
		this->_nh = nh;
		this->_pub = nh.advertise<hinowa_parameters::hinowa_hwi>("/hinowa/hwi2loop", 1000); 

	}

	hwi_publisher::~hwi_publisher()
	{
		
	}

	void hwi_publisher::addData(int levelled, int state_of_trajectory, double loop_hz)
	{
		this->levelled = levelled;
		this->state_of_trajectory = state_of_trajectory; 
		this->loop_hz = loop_hz; 


	}

	void hwi_publisher::publish()
	{
		hinowa_parameters::hinowa_hwi msg;
		msg.header.stamp = ros::Time::now();
		msg.levelled = levelled;
		msg.state_of_trajectory = state_of_trajectory;
		msg.loop_hz = loop_hz;
		_pub.publish(msg);
		ros::spinOnce();		
	}
}

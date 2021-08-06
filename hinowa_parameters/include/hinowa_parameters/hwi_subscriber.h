#ifndef HWI__SUBSCRIBER_H
#define HWI__SUBSCRIBER_H

#include <ros/ros.h>
#include <hinowa_parameters/hinowa_hwi.h>


namespace hinowa_parameters
{
	class hwi_subscriber
	{
		private:
			ros::NodeHandle _nh;
			ros::Subscriber _sub;

		public:
			hwi_subscriber();
			hwi_subscriber(ros::NodeHandle nh);
			~hwi_subscriber();
			void storeData(const hinowa_parameters::hinowa_hwi::ConstPtr& params);
			void subscribe();
			
			int levelled = 0;
			int state_of_trajectory = 0;
			double loop_hz = 0;

	};
}


#endif

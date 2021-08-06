#ifndef CTRL_LOOP__SUBSCRIBER_H
#define CTRL_LOOP__SUBSCRIBER_H

#include <ros/ros.h>
#include <hinowa_parameters/loop_control.h>


namespace hinowa_parameters
{
	class ctrl_loop_subscriber
	{
		private:
			ros::NodeHandle _nh;
			ros::Subscriber _sub;

		public:
			ctrl_loop_subscriber();
			ctrl_loop_subscriber(ros::NodeHandle nh);
			~ctrl_loop_subscriber();
			void storeData(const hinowa_parameters::loop_control::ConstPtr& params);
			void subscribe();
			
			int state = 0;
			std::string active_valve_block = "bottom";
			int ball_valve = 0;
			int reset_levelling = 0;
			int target_aruco_ID = 0;

	};
}


#endif

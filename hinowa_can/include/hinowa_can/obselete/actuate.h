#ifndef ACTUATE_H
#define ACTUATE_H
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include "ros/ros.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <bits/stdc++.h> 
#include <hinowa_can_msgs/valve_control.h>
#include <hinowa_can_msgs/frame.h>
#include <hinowa_can_msgs/frameArray.h>


namespace hinowa_can
{
	class actuate
	{
		public:
			actuate(ros::NodeHandle nh);
			virtual ~actuate();
			void publish();

		private:

			ros::NodeHandle _nh;
			ros::Publisher _actuate_publisher;		
	
	};
}
#endif

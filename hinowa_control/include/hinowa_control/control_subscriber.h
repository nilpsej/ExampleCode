#ifndef HINOWACPP__CONTROL_SUBSCRIBER_H
#define HINOWACPP__CONTROL_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h> 

namespace control
{
	class CONTROL_SUBSCRIBER
	{
		private:
			ros::NodeHandle _nh;
			ros::Subscriber _sub;
			int _PS = 0;
			int _outputPS = 0;
			int _prevPS = 0;
			int _start = 0;
			int _outputStart = 0;
			int _prevStart = 0;
			int _select = 0;
			int _outputSelect = 0;
			int _prevSelect = 0;

		public:
			CONTROL_SUBSCRIBER();
			CONTROL_SUBSCRIBER(ros::NodeHandle nh);
			~CONTROL_SUBSCRIBER();
			void storeData(const sensor_msgs::Joy::ConstPtr& joy);
			std::tuple<int, int, int> subscribe();
	};


}

#endif

#ifndef HINOWA__HINOWA_H
#define HINOWA__HINOWA_H

//#include <sstream>
#include <ros/ros.h>
#include <hinowacpp/segment.h>

namespace hinowacpp
{
	class HINOWA
	{
		private:
		public:
			HINOWA(hinowacpp::CAN_RX_SUBSCRIBER& can_rx, ros::NodeHandle& nh);
			~HINOWA();

			Segment<5> lift;
			Joint getJoint(std::string jointName);
	};
}

#endif

#ifndef HWI__PUBLISHER_H
#define HWI__PUBLISHER_H

#include <ros/ros.h>
#include <hinowa_parameters/hinowa_hwi.h>

namespace hinowa_parameters
{
	class hwi_publisher
	{
		private:
			ros::NodeHandle _nh;
			ros::Publisher _pub;
			int levelled = 0;
			int state_of_trajectory = 0;
			double loop_hz = 0;

			//ros::Time t;

		public:
			hwi_publisher();
			hwi_publisher(ros::NodeHandle nh);
			~hwi_publisher();
			void addData(int levelled, int state_of_trajectory, double loop_hz);
			void publish();
	};
}

#endif

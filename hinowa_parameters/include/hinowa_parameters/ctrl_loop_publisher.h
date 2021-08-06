#ifndef CTRL_LOOP__PUBLISHER_H
#define CTRL_LOOP__PUBLISHER_H
 
#include <ros/ros.h>
#include <hinowa_parameters/loop_control.h>

namespace hinowa_parameters
{
	class ctrl_loop_publisher
	{
		private:
			ros::NodeHandle _nh;
			ros::Publisher _pub;
			int state = 0;
			std::string active_valve_block = "bottom";
			int ball_valve = 0;
			int reset_levelling = 0;
			int target_aruco_ID = 0;

			//ros::Time t;

		public:
			ctrl_loop_publisher();
			ctrl_loop_publisher(ros::NodeHandle nh);
			~ctrl_loop_publisher();
			void addData(int state, int target_aruco_ID, std::string active_valve_block, int ball_valve, int reset_levelling);
			void publish();
	};
}

#endif

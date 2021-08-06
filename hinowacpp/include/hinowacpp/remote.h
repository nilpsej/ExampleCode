#ifndef HINOWACPP__REMOTE_H
#define HINOWACPP__REMOTE_H

//#include <sstream>
//#include <ros/ros.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Float32MultiArray.h>
//#include "std_msgs/String.h"
#include <hinowacpp/can_tx_publisher.h>
#include <sensor_msgs/Joy.h>
//#include <hinowacpp/aruco_feedback.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace hinowacpp
{
	class REMOTE
	{
		private:
			ros::NodeHandle _nh;
			ros::Subscriber _sub;
			//struct can_frame read_frame;

			int _paintTriggerL = 0;
			int _paintTriggerR = 0;			

			int _dPadUp = 0;//switch valve id up
			int _dPadDown = 0;//switch valve id down
			int _dPadLeft = 0;//changes direction of stabiliser input in state 0 (pressing this button moves them downwards).
			int _dPadRight = 0;//enable autonomous track driving based on aruco code feedback when combined with square button. 
			
			int _xButton = 0;//stabiliser raising in state 0, downward effort input for boom control, tuning uses. 
			int _circle; //max effort in up/down when combined with trianlge/x button
			int _triangle;//positive effort input for given joint 
			int _square;//negative effort input for given joint. Also enables autonomous track driving override when combined with d pad right button. 
			int _PS;
			int _Start;
			
			double _leftTrack = 0;
			double _rightTrack = 0;
			int LT_effort = 0;
			int RT_effort = 0;
			
			bool _arucoAligned;
			bool _arucoParallel;
			double T_aligned; //pulled from hinowa_control/config/control.yaml
			double T_parallel; //pulled from hinowa_control/config/control.yaml


			//int _widthOut;
			//int _widthIn;

			int _valveIndex = 0;
			uint16_t _valveList[5] = {0x106, 0x103, 0x102, 0x105, 0x104};
			//uint16_t valveList[7] = {0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97};
			int _effort = 5000;
			std::string _surface_direction = "undefined";

			std::vector<geometry_msgs::TransformStamped> aruco_anchor;
			std::vector<geometry_msgs::TransformStamped> aruco_camera;
			int num_markers = 0;
			geometry_msgs::TransformStamped target_code;

			int _prev_circle = 0;
			double t0;
			double t1;
			double dt = 0.0;
			std::string _engine = "off";
			int target_aruco_ID = 0;
			bool target_aruco_visible = false;
			int prev_dPadRight = 0;
			int prev_square = 0;

		public:
			REMOTE();
			REMOTE(ros::NodeHandle nh);
			~REMOTE();
			void storeData(const sensor_msgs::Joy::ConstPtr& joy);
			void read();
			void engineControl(hinowacpp::CAN_TX_PUBLISHER& can_tx);
			void alternativeControls();
			void tuneTracks();
			void updateArucoFeedback(std::tuple<std::vector<geometry_msgs::TransformStamped>, std::vector<geometry_msgs::TransformStamped>, int> aruco_tuple, int target_aruco_ID);
			void write(hinowacpp::CAN_TX_PUBLISHER& can_tx, int controlState, int ball_valve);
			void autonomousTracks(hinowacpp::CAN_TX_PUBLISHER& can_tx);
	};
}

#endif

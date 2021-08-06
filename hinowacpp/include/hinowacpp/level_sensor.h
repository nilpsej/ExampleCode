#ifndef HINOWACPP__LEVEL_SENSOR_H
#define HINOWACPP__LEVEL_SENSOR_H

//#include <sstream>
//#include <ros/ros.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Float32MultiArray.h>
//#include "std_msgs/String.h"
//#include <sensor_msgs/Joy.h>
#include <hinowacpp/can_rx_subscriber.h>
#include <hinowacpp/can_tx_publisher.h>

namespace hinowacpp
{
	class LEVEL_SENSOR
	{
		private:
			uint16_t _xyID = 0x089;
			//uint16_t _xCANid = 0x098;
			//uint16_t _yCANid = 0x099;
			uint16_t _lightsCANid = 0x090;

			double setPoint = 45.0; //this must match gyro code
			double t = 0.15;//tolerance - change to 0.15

			RX_pair_t _lightsFrame;
			RX_pair_t _frame;
			double _xAngle;
			double _yAngle;
			double _xError = 0;
			double _yError = 0;

			bool _readyForLevelling = false;
			int _errorPriority;
			int _firstSuccess = 1;
			int _resetLevelCheck = 1;

			bool currently_level  = false;
			bool previously_level = false;
			double t0 = 0.0;
			double t1 = 0.0;
			double t_level = 3.0;
			double t0_unlevel[4] = {0.0,0.0,0.0,0.0};
			double t1_unlevel[4] = {0.0,0.0,0.0,0.0};
			int prev_lights[4] = {0,0,0,0};
		public:
			LEVEL_SENSOR();
			LEVEL_SENSOR(uint16_t xyAddress);
			~LEVEL_SENSOR();
			void read(hinowacpp::CAN_RX_SUBSCRIBER& can_rx);
			void reset();
			void level(ros::NodeHandle& nh, hinowacpp::CAN_TX_PUBLISHER& can_tx, int reset_levelling);
			void unlevel(ros::NodeHandle& nh, hinowacpp::CAN_TX_PUBLISHER& can_tx);
			void actuateStabilisers(std::string action, hinowacpp::CAN_TX_PUBLISHER& can_tx, int Hspeed, int Lspeed);

			int is_level = 0;
	};
}

#endif

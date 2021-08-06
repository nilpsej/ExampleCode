#ifndef HINOWACPP__JOINT_H
#define HINOWACPP__JOINT_H

//#include <sstream>
#include "ros/ros.h"
#include <hinowacpp/can_rx_subscriber.h>
#include <hinowacpp/can_tx_publisher.h>
#include <math.h>

namespace hinowacpp
{
	class Joint
	{
		private:
			std::string _name;
			uint16_t _encoderId = 0x0;
			uint16_t _valveId = 0;
			double _minAngle = 0;
			double _maxAngle = 0;
			double _minEncoder = 0;
			double _maxEncoder = 0;
			double _Vmax = 0;
			double _angleError = 0;
			double _percentV = 0;
			double _previousEffort = 0;
			double _encoderRange = 0;
		public:
			
			Joint();
			Joint(uint16_t encoderId, uint16_t valveId);
			~Joint();
			uint16_t getEncoderId();
			void setEncoderId(uint16_t EncoderId);
			uint16_t getValveId();
			void setValveId(uint16_t valveId);
			void setName(std::string);
			std::string getName();
			void setAngleLimits(double min, double max);
			void setEncoderLimits(hinowacpp::CAN_RX_SUBSCRIBER& can_rx, double initAngles[], int i);
			void setEncoderRange(double range);
			void setVelocityLimits(double Vmax);
			void actuate(double velocity, hinowacpp::CAN_TX_PUBLISHER& can_tx, float joint_error, int printEffort);
			void setPreviousEffort(double effort);
			double readAngle(hinowacpp::CAN_RX_SUBSCRIBER& can_rx, bool init);
	};
}

#endif

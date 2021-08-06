//#include <stdlib.h>
//#include <math.h>
//#include <stdexcept>
//#include "ros/ros.h"
#include <hinowacpp/joint.h>

namespace hinowacpp
{
	Joint::Joint()
	{
	}

	Joint::Joint(uint16_t encoderId, uint16_t valveId)
	{
		setEncoderId(encoderId);
		setValveId(valveId);
	}

	Joint::~Joint()
	{

	}
	uint16_t Joint::getEncoderId()
	{
		return this->_encoderId;
	}

	void Joint::setEncoderId(uint16_t encoderId)
	{
		this->_encoderId = encoderId;
	}

	uint16_t Joint::getValveId()
	{
		return this->_valveId;
	}

	void Joint::setValveId(uint16_t valveId)
	{
		this->_valveId = valveId;
	}

	void Joint::setName(std::string name)
	{
		this->_name = name;
	}

	std::string Joint::getName()
	{
		return this->_name;
	}


	double Joint::readAngle(hinowacpp::CAN_RX_SUBSCRIBER& can_rx, bool init)
	{	
		double position = 0.0;
		RX_pair_t RX_pair = can_rx.getData(getEncoderId());

		double encoderValue = 0.0;	
//			std::cout<<_name<<": ";
		if(_name == "joint_1" || _name == "joint_7"){ //Linear encoder case.
			int count = 6;
			for(int i=0;i<4;i++) {
//					printf("%02X", RX_pair.data[i]);
				encoderValue += double(RX_pair.data[3-i]*pow(16,count));
				count -= 2;
			}
			//if(_name == "joint_7"){
				encoderValue = (encoderValue)*0.001; //0.001 for mm to m conversion - accurate as of 31/12/18 (code or 					object dictionary options for zeroing this value at full retraction - see datasheet for relevant info
		}
		else{ //rotary encoder case.
			double encoder1Value = 0.0;
			double encoder2Value = 0.0;	
			int count = 2;
			for(int i=0;i<2;i++) {
//					printf("%02X", RX_pair.data[i]);
				encoder1Value += double(RX_pair.data[i]*pow(16,count));
				encoder2Value += double(RX_pair.data[i+2]*pow(16,count));
				count -= 2;
			}
			encoderValue = 	(encoder1Value+encoder2Value)/2.0;
			encoderValue = encoderValue*0.1; 
			//printf("Rotary encoders: Final: %f, E1: %f, E2: %f.\n", encoderValue, encoder1Value*0.1, encoder2Value*0.1);
			if(encoderValue == 6553.5){
				std::cout<<_name<<" encoder error.";
				throw std::runtime_error("Check magnet mounting & positioning. Exiting for safety.");
			}
		}
//			printf("\n");

//Before HWI even starts, this function is called during the node start up (hinowa_hardware_interface_node.cpp) and does not convert raw encoder measurements to meaningful position values relative to the min and max limits of each joint. It stores the raw value in position instead and uses it to ensure the read value for joint_1 is within the encoders limits. This is to make sure the CAN network is reading sensible data and ready to operate as intended.			
		if(init == true){
			position = encoderValue;
				
		} //end encoder angle return case.

		else{ //in all other cases where position is an angle/prismatic value..
			position = ((encoderValue-_minEncoder)/double(_maxEncoder-_minEncoder))*(_maxAngle-_minAngle)+_minAngle;

			if(position >= _maxAngle) position = _maxAngle;
			else if(position <= _minAngle) position = _minAngle;
/*		
			if(_name == "joint_6"){			
				std::cout<<"|Encoder frame read for: "<<_name<<"|";				
				printf(" canID: %X, Encoder value: %lf, joint position: %lf.", _encoderId, encoderValue, position);
				printf("\n--------------\n");

			}
*/

		}//end joint angle return case.
		return position;

/*
	else{ //No data/incomplete data has been read and stored in read_frame.data, throw error and exit.
		throw std::runtime_error("CAN Read Error during joint position read. Exiting for safety.");
	}
*/
	}

	void Joint::actuate(double velocity, hinowacpp::CAN_TX_PUBLISHER& can_tx, float joint_error, int printEffort)
	{
		_angleError = (joint_error/(abs(_minAngle)+abs(_maxAngle)))*100.0;
		_percentV = (velocity/_Vmax)*100.0;

		double effort = 0;
		//effort = ((velocity+(_Vmax/0.85))/((_Vmax/0.85)+(_Vmax/0.85))*20000)-10000; //if joint limits are scaled by 0.85 in yaml, use this.
		effort = ((velocity+(_Vmax))/((_Vmax)+(_Vmax))*20000)-10000;

		//if(_name == "joint_7"){
			//std::cout<<_name;	
			//printf("-> Velocity: %lf, Effort: %lf\n", velocity, effort);
		//}

		if (effort > 10000) effort = 10000;
		if (effort < -10000) effort = -10000;
		int r_effort = round(effort);

		std::string sendFlag = "sent";
		can_tx.addData(_valveId, r_effort);

		if(printEffort == 1){
			//std::cout<<_name<<": Velocity "<<sendFlag<<": ";
			std::cout<<_name<<": V: ";
			printf("[%04lf](%02f%%), Effort: [%i], Angle error: [%02f%%].\n",velocity, _percentV , r_effort, _angleError);
		}		

		setPreviousEffort(r_effort);
	}


	void Joint::setPreviousEffort(double effort)
	{
		this->_previousEffort = effort;
	}
/*
	double Joint::getPreviousEffort()
	{
		return this->_previousEffort;
	}
*/
	void Joint::setAngleLimits(double min, double max)
	{
		this->_minAngle = min;
		this->_maxAngle = max;
	}
	
	void Joint::setEncoderLimits(hinowacpp::CAN_RX_SUBSCRIBER& can_rx, double initAngles[], int i)
	{
		if(_name == "joint_1"){
			_minEncoder = initAngles[i]-(_encoderRange*0.5);
			_maxEncoder = initAngles[i]+(_encoderRange*0.5);
		}
    		else if(_name == "joint_6" || _name == "joint_8"){
			_minEncoder = initAngles[i];
			_maxEncoder = initAngles[i]-_encoderRange;
		}
		else if(_name == "joint_4"){
			_maxEncoder = initAngles[i];
			_minEncoder = initAngles[i]+_encoderRange;
		}		
		else{//Joint 7
			_minEncoder = initAngles[i];
			_maxEncoder = initAngles[i]+_encoderRange;
		}
		double currentAngle = readAngle(can_rx, true);		
		std::cout<<_name;
		//current value currently doesn't work, is an issue with the start-up time of the CAN BCM in comparision to everything else. 
		printf("startup info: Home: %lf, current: %lf, encoder range: %lf to %lf.\n", initAngles[i], currentAngle, _minEncoder, _maxEncoder);

	}

	void Joint::setEncoderRange(double range)	
	{
		this->_encoderRange = range;
	}

	void Joint::setVelocityLimits(double Vmax)
	{
		this->_Vmax = Vmax;
	}


}


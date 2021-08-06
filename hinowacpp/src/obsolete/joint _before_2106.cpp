//#include <stdlib.h>
//#include <math.h>
//#include <stdexcept>
#include "ros/ros.h"
#include <hinowacpp/joint.h>
#include <hinowacpp/can_interface.h>
#include <hinowacpp/joint_subscriber.h>

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

/*
//not currently used - test useability if filtering of encoders is needed to acheive joint settling to steady state.
	double Joint::_filterAngle(double angle)
	{
		_angleReads = _angleReads + 1;

		// put value at front of array
		for (int i = _filterPrevious - 1; i > 0; i--) {
			_previousAngles[i] = _previousAngles[i - 1];
		}
		_previousAngles[0] = angle;


		int filterIterations = _filterPrevious;
		if (_angleReads < _filterPrevious) {
			filterIterations = _angleReads;
		}

		double angleSum = 0;
		for (int i = 0; i < filterIterations; i++) {
			angleSum = angleSum + _previousAngles[i];
		}

		double filterResult = angleSum / (filterIterations * 1.0);

		//ROS_INFO("%f, %f, %f, %i", angle, angleSum, filterResult, filterIterations);

		return filterResult;
	}
*/

	double Joint::readAngle(hinowacpp::CAN& can, bool init, hinowacpp::JOINT_SUBSCRIBER& joint_subscriber)
	{		
		double position = 0;
		if(_name != "joint_end_bottom"){ //joint_end_bottom (rotation of end effector is not currently controlled).
			can.filter(_encoderId);
			int result = can.readData(_encoderId, read_frame);
			if (result == 1) { //can data has been read and stored in read_frame.data, continuing.
				double encoderValue = 0;
				
//				std::cout<<_name<<": ";

				if(_name == "joint_1" || _name == "joint_7"){ //Linear encoder case.
					int count = 6;
					for(int i=0;i<4;i++) {
//						printf("%02X", read_frame.data[i]);
						encoderValue += double(read_frame.data[3-i]*pow(16,count));
						count -= 2;
					}
					//if(_name == "joint_7"){
					encoderValue = (encoderValue)*0.001; //0.001 for mm to m conversion - accurate as of 31/12/18 (code or 						object dictionary options for zeroing this value at full retraction - see datasheet for relevant info
				}
				else{ //rotary encoder case.
					int count = 2;
					for(int i=0;i<2;i++) {
//						printf("%02X", read_frame.data[i]);
						encoderValue += double(read_frame.data[i]*pow(16,count));
						count -= 2;
					}	
					encoderValue = encoderValue*0.1; 
					if(encoderValue == 6553.5){
						std::cout<<_name<<" encoder error.";
						throw std::runtime_error("Check magnet mounting & positioning. Exiting for safety.");
					}
				}
//				printf("\n");
				if(init == true){ //Before HWI even starts, this function is called during the node start up (hinowa_hardware_interface_node.cpp) and does not convert raw encoder measurements to meaningful position values relative to the min and max limits of each joint. It stores the raw value in position instead and uses it to ensure the read value for joint_1 is within the encoders limits. This is to make sure the CAN network is reading sensible data and not sending encoder readings that could result in dangerous, unpredictable movement on start up. 
					position = encoderValue;
					
				} //in all other cases..
				else{
					//if(encoderValue < _minEncoder) encoderValue = _minEncoder;
					//if(encoderValue > _maxEncoder) encoderValue = _maxEncoder;
					position = ((encoderValue-_minEncoder)/double(_maxEncoder-_minEncoder))*(_maxAngle-_minAngle)+_minAngle;
					//if(_name == "joint_6" || _name == "joint_8") position = _maxAngle-(position-_minAngle);
					//position = _filterAngle(position);
					position = round(position*100.0)/100.0; //uncomment this line for less position accuracy (ignore very small encoder changes - commented on the 13/03 (after testing usefullness to reduce hunting as an attempt to stop j2 from being unfixed)	

//uncomment below for print out	
					if(position >= _maxAngle) position = _maxAngle;
					else if(position <= _minAngle) position = _minAngle;
				
					//if(_name == "joint_8"){			
					//std::cout<<"|Encoder frame read for: "<<_name<<"|";				
					//printf(" canID: %X, Encoder value: %lf, joint position: %lf.", _encoderId, encoderValue, position);
					//printf("\n--------------\n");
					//}
				}
				return position;
				//old joint.readAngle code commented out at the bottom of this file.
			}
			else { //No data/incomplete data has been read and stored in read_frame.data, throw error and exit.
				throw std::runtime_error("CAN Read Error during joint position read. Exiting for safety.");
			}
		}
		else{
			if(_name == "joint_end_bottom"){
				position = joint_subscriber.subscribe(_name);
			}
			/*
			else if(_name == "joint_1"){
				position = joint_subscriber.subscribe(_name);
			}
			*/
			return position;
		}
	}

	void Joint::actuate(double velocity, hinowacpp::CAN& can)
	{
		if((_name != "joint_end_bottom")){
			double effort = 0;
			effort = ((velocity+_Vmax)/(_Vmax+_Vmax)*20000)-10000;

			//if(_name == "joint_7"){
				//std::cout<<_name;	
				//printf("-> Velocity: %lf, Effort: %lf\n", velocity, effort);
			//}

			if (effort > 10000) effort = 10000;
			if (effort < -10000) effort = -10000;
			int r_effort = round(effort);

//Review the effect of the following if block on performance - is it a good thing that the same effort value wont be sent back to back? does the value being asked of each joint vary enough that we don't end up with points in the execution where values aren't being sent?
			std::string sendFlag = "sent";
			int result = can.writeData(_valveId, r_effort);
			/*
			if(r_effort != _previousEffort){			
				int result = can.writeData(_valveId, r_effort);
				sendFlag = "sent";
			}*/

			//uncomment below for print out
			if(_name == "joint_1"){
				std::cout<<_name<<": Velocity "<<sendFlag<<": ";	
					printf("[%lf], Effort: [%lf], rounded to: [%i].\n",velocity, effort, r_effort);
			}		

			setPreviousEffort(r_effort);
//--------------------------------------------------------------------------------------------
/*
			//double min_effort = -26;//-13.962177;
			//double max_effort = 26;//13.962177;
			//printf("------------------------------------------------------\n");
			//std::cout<<_name;
			//printf(" effort command before scaling: %.2lf.\n", effort);
			double effort;
			double r_velocity = (round(velocity*1000.0)/1000.0);  //3 decimal places
			//printf("rounded velocity: %lf", r_velocity);
			if(r_velocity == 0) velocity = 0;
			if(r_velocity > 0) effort = _upperDZ+(double(velocity-_VminUp)/double(_Vmax-_VminUp))*(10000.0-_upperDZ);
			else if(r_velocity < 0) effort = _lowerDZ+(double(velocity-_VminDown)/double(_Vmax-_VminDown))*(10000.0-_lowerDZ);
			else effort = 0;
			if(effort > 0 && effort < _upperDZ) effort = 0;
			else if(effort < 0 && effort > _lowerDZ) effort = 0;
			
			//currently effort values below deadband are still sent but shouldnt result in any movement - 
			
			//if ((_name == "joint_1") && ((round(effort) == 10001) || (round(effort) == 9759.88))) effort = 0;
			//effort = (double(effort-min_effort)/double(max_effort-min_effort))*(20000.00)-10000.00;
*/
//			if (effort > 10000) effort = 10000;
//			if (effort < -10000) effort = -10000;
//			int r_effort = round(effort);

			//int threshold = 200; //monitor the effectiveness of this value
			//call steadyStateCheck and overide lower and upper DZ - apply new value for n amount times before shrinking again etc	
			//this-> _effortList = steadyStateCheck(r_effort);
			/*
			if(r_effort < -threshold && r_effort > _lowerDZ) r_effort = _lowerDZ;
			if(r_effort > threshold && r_effort < _upperDZ) r_effort = _upperDZ;
				if (r_effort < threshold && r_effort > -threshold) r_effort = 0;
			*/
		
//Review the effect of the following if block on performance - is it a good thing that the same effort value wont be sent back to back? does the value being asked of each joint vary enough that we don't end up with points in the execution where values aren't being sent?
/*
			std::string sendFlag = "not sent";
			if(r_effort != _previousEffort){			
				int result = can.writeData(_valveId, r_effort);
				sendFlag = "sent";
			}
*/
			//uncomment below for print out
			//if(_name == "joint_4"){
				//std::cout<<_name<<": Velocity "<<sendFlag<<": ";	
				//printf("[%lf], rounded to: [%f]. Effort: [%lf], rounded to: [%i].\n",velocity, r_velocity, effort, r_effort);
			//}		

//			setPreviousEffort(r_effort);
		}
			
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
	
	void Joint::setEncoderLimits(hinowacpp::CAN& can, double staticInitAngles[], int i, hinowacpp::JOINT_SUBSCRIBER& joint_subscriber)
	{
		if(_name == "joint_1"){
			_minEncoder = staticInitAngles[i]-(_encoderRange*0.5);
			_maxEncoder = staticInitAngles[i]+(_encoderRange*0.5);
		}//(initAngle-(angleRange[i]*0.5)), (initAngle+(angleRange[i]*0.5)));}
    		else if(_name == "joint_6" || _name == "joint_8"){
			_minEncoder = staticInitAngles[i]-_encoderRange;
			_maxEncoder = staticInitAngles[i];
		}//hinowa_.setupEncoder(joint_names_[i], initAngle-angleRange[i], initAngle);}		
		else{
			_minEncoder = staticInitAngles[i];
			_maxEncoder = staticInitAngles[i]+_encoderRange;
		}//hinowa_.setupEncoder(joint_names_[i], initAngle, initAngle+angleRange[i]);}
		//_minEncoder = staticInitAngles[i];	
		//_maxEncoder = staticInitAngles[i]+_encoderRange;
		double initAngle = readAngle(can, true, joint_subscriber);		
			std::cout<<_name;
		printf(" initial Encoder value: %lf, encoder range: %lf to %lf.\n", initAngle, _minEncoder, _maxEncoder);

/*
		if(dynamicEncoders == true){
			double initAngle = readAngle(can, true, joint_subscriber);
			if(_name == "joint_1"){
				_minEncoder = initAngle-(_encoderRange*0.5);
				_maxEncoder = initAngle+(_encoderRange*0.5);
			}//(initAngle-(angleRange[i]*0.5)), (initAngle+(angleRange[i]*0.5)));}
    			else if(_name == "joint_6" || _name == "joint_8"){
				_minEncoder = initAngle-_encoderRange;
				_maxEncoder = initAngle;
			}//hinowa_.setupEncoder(joint_names_[i], initAngle-angleRange[i], initAngle);}		
			else{
				_minEncoder = initAngle;
				_maxEncoder = initAngle+_encoderRange;
			}//hinowa_.setupEncoder(joint_names_[i], initAngle, initAngle+angleRange[i]);}

			std::cout<<_name;
			printf(" initial Encoder value: %lf, encoder range: %lf to %lf.\n", initAngle, _minEncoder, _maxEncoder);
		}


		else if(dynamicEncoders == false){
			if(_name == "joint_1"){
				_minEncoder = staticInitAngles[i]-(_encoderRange*0.5);
				_maxEncoder = staticInitAngles[i]+(_encoderRange*0.5);
			}//(initAngle-(angleRange[i]*0.5)), (initAngle+(angleRange[i]*0.5)));}
    			else if(_name == "joint_6" || _name == "joint_8"){
				_minEncoder = staticInitAngles[i]-_encoderRange;
				_maxEncoder = staticInitAngles[i];
			}//hinowa_.setupEncoder(joint_names_[i], initAngle-angleRange[i], initAngle);}		
			else{
				_minEncoder = staticInitAngles[i];
				_maxEncoder = staticInitAngles[i]+_encoderRange;
			}//hinowa_.setupEncoder(joint_names_[i], initAngle, initAngle+angleRange[i]);}
			//_minEncoder = staticInitAngles[i];	
			//_maxEncoder = staticInitAngles[i]+_encoderRange;
			double initAngle = readAngle(can, true, joint_subscriber);		

			std::cout<<_name;
			printf(" initial Encoder value: %lf, encoder range: %lf to %lf.\n", initAngle, _minEncoder, _maxEncoder);
		}
	*/

	}

	void Joint::setDeadzone(int lower, int upper)
	{
		this->_lowerDZ = lower;
		this->_upperDZ = upper;
	}

	void Joint::setEncoderRange(double range)
	{
		this->_encoderRange = range;
	}

	void Joint::setVelocityLimits(double Vmax, double VminUp, double VminDown)
	{
		this->_Vmax = Vmax;
		this->_VminUp = VminUp;
		this->_VminDown = VminDown;
	}


}


/*
				//double magnitude = _maxAngle-_minAngle;
				//if(_maxAngle<0 && _minAngle<0) double magnitude = abs(abs(_maxAngle)-abs(_minAngle));
				//printf("%lf, max %lf, min %lf\n", magnitude, _maxAngle, _minAngle);
				if(position < _minEncoder) position = _minEncoder;
				if(position > _maxEncoder) position = _maxEncoder;
				double angle = ((position-_minEncoder)/double(_maxEncoder-_minEncoder))*(_maxAngle-_minAngle)+_minAngle;
				if(_name == "joint_6" || _name == "joint_8") angle = _maxAngle-(angle-_minAngle);
				//double angle = ((position/10000.0)*magnitude)+_minAngle;
				//double filteredAngle = Joint::_filterAngle(angle);
				//if(angle < _minAngle) angle = _minAngle;
				//if(angle > _maxAngle) angle = _maxAngle;
				//replace the above with the below where min_effort/max_effort will be encoder min and max (exp to find)
				//effort = ((effort-min_effort)/(max_effort-min_effort))*(20000)-10000;
				if(_name == "joint_7"){			
					std::cout<<_name;
					printf(": Angle: %lf, Position: %d\n", angle, position);
				}
				//test the useability of code similar to the below - will it smooth the pot?

//two following lines were uncommented before 31/12/18 - see if they make a difference with new encoders
				//angle = _filterAngle(angle);
				//angle = round(angle*100.0)/100.0;
				//printf("angle %lf\n",angle);
				//angle += angleOffset;
				//if (angle > PI) angle -= TAU;
				//if (angle < -PI) angle += TAU;
				//angle *= readRatio;
				//double angle = position;
				return angle;
*/

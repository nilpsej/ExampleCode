#include <hinowacpp/level_sensor.h>
#include <sensor_msgs/Joy.h>

namespace hinowacpp
{

	LEVEL_SENSOR::LEVEL_SENSOR()
	{

	}

	LEVEL_SENSOR::LEVEL_SENSOR(uint16_t xyAddress)
	{
		this-> _xyID = xyAddress;
	}

	LEVEL_SENSOR::~LEVEL_SENSOR()
	{

	}
	
	void LEVEL_SENSOR::read(hinowacpp::CAN_RX_SUBSCRIBER& can_rx)
	{
		//Read status of light Digital inputs to Danfoss controller, will be used to check if stablisers are ready to commence leveling)
		_lightsFrame = can_rx.getData(_lightsCANid);

		//printf("Status of stablisers: ");
		if(_lightsFrame.data[0] && _lightsFrame.data[1] && _lightsFrame.data[2] && _lightsFrame.data[3]){//s1, s2, s3, s4 check
		//	printf("Ready for leveling.\n");	
			_readyForLevelling = true; //all stabliser lights are on, set level ready flag to true		
		}
		else{
		
			printf("Not all stablisers are primed with lights on, lowering remaining stablisers with: ");
			_readyForLevelling = false;//not all lights are on, continue first step of moving all stablisers downwards
		}
	
		//Read and assign either x or y axis angle of gyro sensor (depending on input variable axis). 
		double Xangle = 0.0;
		double Yangle = 0.0;
		_frame = can_rx.getData(_xyID);

		int count = 3;
		for(int i=0;i<4;i++) {
			//printf("%02X", _frame.data[i]);
			Xangle += double(_frame.data[3-i]*pow(16,count));
			Yangle += double(_frame.data[7-i]*pow(16,count));
			count--;
		}
		_xAngle = Xangle*0.01;
		_yAngle = Yangle*0.01;
		//printf("Level sensor -  X Angle: %lf, Y Angle: %lf\n", _xAngle, _yAngle);

		
/*
		if(axis == 'x'){
			_frame = can_rx.getData(_xCANid);
			//printf("Reading x axis. Hex: ");
		}
		else if(axis == 'y'){
			_frame = can_rx.getData(_yCANid);
			//printf("Reading y axis. Hex: ");
		}
		int count = 3;
		for(int i=0;i<4;i++) {
			//printf("%02X", _frame.data[i]);
			angle += double(_frame.data[3-i]*pow(16,count));
			count -= 1;
		}
		printf(", ");
		angle = angle*0.01;
		//printf("Angle: %lf\n", angle);
		
		if(axis == 'x') _xAngle = angle;
		else if(axis == 'y') _yAngle = angle;
*/
	}
	void LEVEL_SENSOR::reset()
	{
		_firstSuccess = 0;
		currently_level = false;
		previously_level = false;
		t0 = 0.0; t1 = 0.0;
		is_level = 0;
		for(int i = 0; i < 4; i++){
			t0_unlevel[i] = 0.0;
			t1_unlevel[i] = 0.0;
		}
		
	}

	void LEVEL_SENSOR::level(ros::NodeHandle& nh, hinowacpp::CAN_TX_PUBLISHER& can_tx, int reset_levelling)	
	{
		int speed = 5000;
		int levelTotal = 0;

		if(_readyForLevelling){
			if(reset_levelling) reset();

//TODO:check this functions as intended, just more testing.
			if((_xAngle <= (setPoint+t)) && (_xAngle >= (setPoint-t)) && (_yAngle <= (setPoint+t)) && (_yAngle >= (setPoint-t)))
				currently_level = true;

			if(previously_level)
				if((_xAngle > (setPoint+(t*2))) || (_xAngle < (setPoint-(t*2))) || (_yAngle > (setPoint+(t*2))) || (_yAngle < (setPoint-(t*2)))) currently_level = false;
			

			if(currently_level && !previously_level) t0 = ros::Time::now().toSec();
			else if(currently_level && previously_level) t1 = ros::Time::now().toSec();
			else t1 = t0;
			

			if((t1-t0) > t_level) is_level = 1;
			else is_level = 0;
			previously_level = currently_level;

			if(!is_level){
				_firstSuccess = 1; 
				_xError = _xAngle - setPoint;
				_yError = _yAngle - setPoint;
				printf("Errors x: %lf, y: %lf.\n", _xError, _yError);
				if((fabs(_xError) <= fabs(t)) && (fabs(_yError) <= fabs(t))){
					printf("Within tolerance level tolerance for: %f seconds.\n", t1-t0);
					can_tx.addData(0x91, 0);
					can_tx.addData(0x92, 0);
					can_tx.addData(0x95, 0);
					can_tx.addData(0x96, 0);
				}
				else{
					actuateStabilisers("level", can_tx, 5000, -3800); //find a happy balance for these two values (2nd stage of level).
				}
		
			} //end of if not level
			else{// considered level
				can_tx.addData(0x91, 0);
				can_tx.addData(0x92, 0);
				can_tx.addData(0x95, 0);
				can_tx.addData(0x96, 0);
				if(_firstSuccess){
					printf("Robot levelled. Waiting for control state to be advanced.\n");
					is_level = 1;
				}
				_firstSuccess = 0; //no longer first entry
			}

		}//end of ready for levelling if statement, else is to keep moving down till lights come on.
		else{
			_xError = _xAngle - setPoint;
			_yError = _yAngle - setPoint;

			actuateStabilisers("level", can_tx, 8000, 7500); //8000, 7500
		}
	}


	void LEVEL_SENSOR::unlevel(ros::NodeHandle& nh, hinowacpp::CAN_TX_PUBLISHER& can_tx)	
	{
		actuateStabilisers("unlevel", can_tx, -4000, -4000);
	}

	void LEVEL_SENSOR::actuateStabilisers(std::string action, hinowacpp::CAN_TX_PUBLISHER& can_tx, int Hspeed, int Lspeed)	
	{

		uint16_t idList[4] = {0x91, 0x92, 0x95, 0x96};
		uint16_t speeds[4] = {};

		double max = _xError;
		if(fabs(_yError) > fabs(_xError)) max = _yError;
		
		if(max == _xError){
			if(_xError > 0) _errorPriority = 1;
			else _errorPriority = 2;	
		}
		else if(max == _yError){
			if(_yError > 0) _errorPriority = 3;
			else _errorPriority = 4;
		}
		
		switch(_errorPriority){
			case 1 : speeds[0] = Lspeed; speeds[1] = Hspeed; speeds[2] = Hspeed; speeds[3] = Lspeed;
				 if(_readyForLevelling) printf("Raising back.\n");
				 else printf("Back side priority.\n");
				 break;
			case 2 : speeds[0] = Hspeed; speeds[1] = Lspeed; speeds[2] = Lspeed; speeds[3] = Hspeed;
				 if(_readyForLevelling) printf("Raising front.\n");
				 else printf("Front side priority.\n");
				 break;
			case 3 : speeds[0] = Hspeed; speeds[1] = Hspeed; speeds[2] = Lspeed; speeds[3] = Lspeed;
				 if(_readyForLevelling) printf("Raising left side.\n");
				 else printf("Left side priority.\n");
				 break;
			case 4 : speeds[0] = Lspeed; speeds[1] = Lspeed; speeds[2] = Hspeed; speeds[3] = Hspeed;
				 if(_readyForLevelling) printf("Raising right side.\n");
				 else printf("Right side priority.\n");
				 break;
		}
		for(int i = 0; i < 4; i++){ //for each stabiliser, apply the updated speed array determined by the switch case.
			if(action == "level"){
				if(_lightsFrame.data[i] == 0) can_tx.addData(idList[i], -speeds[i]);//continue bringing stabilisers down. 
				else if(_lightsFrame.data[i] == 1){
					if(!_readyForLevelling) can_tx.addData(idList[i], 0);
					else if(_readyForLevelling) can_tx.addData(idList[i], -speeds[i]);
				}
			}
			else if(action == "unlevel"){
				if(_lightsFrame.data[i] == 1) can_tx.addData(idList[i], -speeds[i]); //weight sensors still under load
				else if(_lightsFrame.data[i] == 0){ //weight sensors no longer under load - lights off
					if(prev_lights[i]) t0_unlevel[i] = ros::Time::now().toSec(); //just switched, record t0
					else{
						t1_unlevel[i] = ros::Time::now().toSec(); //keep recording latest time
						//printf("Stabilisers #%d time:%f.\n", i, t1_unlevel[i]-t0_unlevel[i]);
					}
					if((t1_unlevel[i]-t0_unlevel[i]) < 10.0) can_tx.addData(idList[i], -speeds[i]); //keep raising stabilisers for ground clearance
					else can_tx.addData(idList[i], 0); //if latest time - t0 exceeds 5s, stop
				}
				prev_lights[i] = _lightsFrame.data[i];
			}
  		}
	}	
}

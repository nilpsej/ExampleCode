//#include <stdlib.h>
//#include <math.h>
//#include <stdexcept>
//#include <ros/ros.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Float32MultiArray.h>
//#include "std_msgs/String.h"
//#include <geometry_msgs/Twist.h>
#include <hinowacpp/remote.h>
#include <sensor_msgs/Joy.h>


namespace hinowacpp
{

	REMOTE::REMOTE()
	{

	}

	REMOTE::REMOTE(ros::NodeHandle nh)
	{
		this->_nh = nh;
		_nh.getParam("/surface/direction", _surface_direction);
		_nh.getParam("/aruco/translation_align_tolerance", T_aligned);
		_nh.getParam("/aruco/rotation_align_tolerance", T_parallel);
	}

	REMOTE::~REMOTE()
	{

	}
	
	void REMOTE::storeData(const sensor_msgs::Joy::ConstPtr& joy)
	{
		_paintTriggerL = joy->buttons[4];
		_paintTriggerR = joy->buttons[5];

//Boom operations
		_dPadUp = joy->buttons[13];
		_dPadDown = joy->buttons[14];
		_triangle = joy->buttons[2];
		_xButton = joy->buttons[0]; //used for base as well
		_circle = joy->buttons[1];
		_square = joy->buttons[3]; //used for base as well

//Base operations
		_leftTrack = joy->axes[1]; //left TS
		_rightTrack = joy->axes[4]; //right TS
		//_xButton = joy->buttons[0]; //declared above but used for both
		_dPadLeft = joy->buttons[15];
		_dPadRight = joy->buttons[16];
		//_widthOut = joy->buttons[13];
		//_widthIn = joy->buttons[14];

		//_Start = joy->buttons[9]; //decide what buttons to use here for motor start
		//_PS = joy->buttons[10];
	}


	void REMOTE::read()	
	{
		this->_sub = _nh.subscribe<sensor_msgs::Joy>("/joy", 1000, &REMOTE::storeData, this);	
	}

	void REMOTE::engineControl(hinowacpp::CAN_TX_PUBLISHER& can_tx){
//this works for now - change so it responds to a RX ID from the rpm sensor that determines whether the engine is already on or off. From there, a simple if else will allow toggling of the current state. Dont press engine start if code has been started with engine running for now. 	
		if(_circle && !_prev_circle) t0 = ros::Time::now().toSec();
		else if(_circle && _prev_circle && (dt < 1.0)){
			t1 = ros::Time::now().toSec();
			dt = t1-t0;
			//printf("dt: %f.\n", dt);
			if((dt > 1.0) && (_engine == "off")){
				_engine = "on";
				printf("STARTING ENGINE.\n");
				//can_tx.addData(0x050, 1);
			}
			else if((dt > 1.0) && (_engine == "on")){
				_engine = "off";
				printf("TURNING ENGINE OFF.\n");
				//can_tx.addData(0x050, 0);
			}
		}
		else if(!_circle && _prev_circle){
			dt = 0.0;
		}
		_prev_circle = _circle;
	}
	
	void REMOTE::write(hinowacpp::CAN_TX_PUBLISHER& can_tx, int controlState, int ball_valve)	
	{
		//engineControl(can_tx);

		if(controlState > -1){

			if(_paintTriggerL && _paintTriggerR){
				can_tx.addData(0x60, 1);
				//can.writeData(0x60, -1);
			}
			else if(ball_valve) can_tx.addData(0x60, 1);
			else can_tx.addData(0x60, 0);
		}

		if(controlState == -1){ 
		//not in typical code build. Change to 0 if want it to override first state (and change next else state to -1).
			alternativeControls();
		}


		if(controlState == 0){ //Base control, intended initial state.
			bool tuning_tracks = false; //if tuning, set to true. Do not have this set true when operating.

//------------------------------------------Stablisers---------------------------------------------//

			//If stabliser reverse button is pressed (left on D-pad), change sign of effort.			
			int stabliserDir = 1;
			if(_dPadLeft == 1) stabliserDir = -1;
			//Send CAN commands for each. 
			can_tx.addData(0x91, 6000*stabliserDir*_xButton);	
			can_tx.addData(0x92, 6000*stabliserDir*_xButton);
			can_tx.addData(0x95, 6000*stabliserDir*_xButton);
			can_tx.addData(0x96, 6000*stabliserDir*_xButton);


//--------------------------------------------Tracks-----------------------------------------------//
			if(_dPadRight && _square) autonomousTracks(can_tx);
			else{
				if(!tuning_tracks){
					LT_effort = _leftTrack*10000*0.5;//both effort values are scaled to 10000, 0.5 scalar is to set max speed to 50%
					RT_effort = _rightTrack*10000*0.5;

					//Send CAN messages to tracks.
					can_tx.addData(0x93, LT_effort);//can.writeData(0x93, LT_effort);
					can_tx.addData(0x94, RT_effort);//can.writeData(0x94, RT_effort);
			
					if(LT_effort != 0.0 || RT_effort != 0.0) printf("Left track effort: %d, right track effort: %d.\n", LT_effort, RT_effort);
				}

				else if(tuning_tracks) tuneTracks();
			}
		prev_dPadRight = _dPadRight; prev_square = _square; //track previous state of buttons to enable autoTracks print outs
		}
	}

	void REMOTE::alternativeControls(){
/*
		bool tuning = true;
		if(!tuning){
			int speed = 6000;
			if((_triangle == 0 && _xButton == 0) || (_triangle == 1 && _xButton == 1)) _effort = 0;
			else if(_triangle == 1 && _xButton == 0) _effort = speed;
			else if(_triangle == 0 && _xButton == 1) _effort = -1*speed;

			if((_dPadUp == 0 && _dPadDown == 0) || (_dPadUp == 1 && _dPadDown == 1)) _valveIndex = _valveIndex;
			if(_dPadUp == 1 && _dPadDown == 0) _valveIndex++;
			else if(_dPadUp == 0 && _dPadDown == 1) _valveIndex--;

			if(_valveIndex < 0) _valveIndex = 4;
			else if(_valveIndex > 4) _valveIndex = 0;
			printf("Current effort: %i, current valve: %X.\n", _effort, _valveList[_valveIndex]);
			can_tx.addData(_valveList[_valveIndex], _effort);//can.writeData(_valveList[_valveIndex], _effort);
			}

		if(tuning){
			int appliedEffort = 0;
			if((_triangle == 0 && _xButton == 0) || (_triangle == 1 && _xButton == 1)) appliedEffort = _effort*0;
			else if(_triangle == 1 && _xButton == 0) appliedEffort = 1*_effort;
			else if(_triangle == 0 && _xButton == 1) appliedEffort = -1*_effort;
			if(_square == 0 && _dPadUp == 1 && _dPadDown == 0) _effort+= 200;
			else if(_square == 0 && _dPadUp == 0 && _dPadDown == 1) _effort-= 200;
			else if(_square == 1 && _dPadUp == 1 && _dPadDown == 0) _effort+= 1;
			else if(_square == 1 && _dPadUp == 0 && _dPadDown == 1) _effort-= 1;
			if(_circle == 1 && _triangle == 1) appliedEffort = 10000;
			if(_circle == 1 && _xButton == 1) appliedEffort = -10000;
			uint16_t valve = 0x102;
			printf("current effort value: %i, appliedEffort: %i, current valve: %X.\n", _effort, appliedEffort, valve);
			can_tx.addData(valve, appliedEffort);//can.writeData(valve, appliedEffort);
		}
*/
	}

	void REMOTE::tuneTracks(){
/*
		int trackAppliedEffort = 0;
		if((_triangle == 0 && _xButton == 0) || (_triangle == 1 && _xButton == 1)) trackAppliedEffort = _effort*0;
		else if(_triangle == 1 && _xButton == 0) trackAppliedEffort = 1*_effort;
		else if(_triangle == 0 && _xButton == 1) trackAppliedEffort = -1*_effort;
		if(_square == 0 && _dPadUp == 1 && _dPadDown == 0) _effort+= 200;
		else if(_square == 0 && _dPadUp == 0 && _dPadDown == 1) _effort-= 200;
		else if(_square == 1 && _dPadUp == 1 && _dPadDown == 0) _effort+= 1;
		else if(_square == 1 && _dPadUp == 0 && _dPadDown == 1) _effort-= 1;
		if(_circle == 1 && _triangle == 1) trackAppliedEffort = 10000;
		if(_circle == 1 && _xButton == 1) trackAppliedEffort = -10000;
		uint16_t trackValve = 0x94;
		printf("current effort value: %i, appliedEffort: %i, current valve: %X.\n", _effort, trackAppliedEffort, trackValve);
		can_tx.addData(trackValve, trackAppliedEffort);//can.writeData(trackValve, trackAppliedEffort);
*/
	}

	void REMOTE::updateArucoFeedback(std::tuple<std::vector<geometry_msgs::TransformStamped>, std::vector<geometry_msgs::TransformStamped>, int> aruco_tuple, int target_aruco_ID){
		this->num_markers = std::get<2>(aruco_tuple);
		this->target_aruco_ID = target_aruco_ID;

		target_aruco_visible = false;
		//printf("num_markers: %d.\n", num_markers);
		if(num_markers > 0){
			this->aruco_anchor = std::get<0>(aruco_tuple);
			this->aruco_camera = std::get<1>(aruco_tuple);
			for(int i = 0; i < num_markers; i++){
				if(aruco_camera.at(i).header.seq == target_aruco_ID){
					this->target_code = aruco_camera.at(i);
					target_aruco_visible = true;
				}
			}
		}
	}


	void REMOTE::autonomousTracks(hinowacpp::CAN_TX_PUBLISHER& can_tx)
	{
		if(!target_aruco_visible){
			if(!prev_dPadRight || !prev_square){
				printf("\033[0;31m");
				if(num_markers > 0){
					printf("Visible ArUco codes do not include the current target ID: %d. Check codes are arranged as intended and/or reposition the machine manually to ensure the target code is within the FOV.\n", target_aruco_ID);
				}
				else{
					printf("No ArUco codes currently visible. Reposition the machine manually to ensure the target code with ID: %d is within the FOV.\n", target_aruco_ID);
				}
				printf("\033[0m");
			}
		}
		else{ //target code visible - carry out alignment routine.
		tf2::Vector3 V_aruco(target_code.transform.translation.x, target_code.transform.translation.y, target_code.transform.translation.z);
			tf2::Quaternion Q_aruco;
			tf2::convert(target_code.transform.rotation, Q_aruco);
			tf2::Vector3 arucoRPY;
			tf2::Matrix3x3(Q_aruco).getRPY(arucoRPY[0], arucoRPY[1], arucoRPY[2], 2);

			//printf("Aruco: V - X: %f, Y: %f, Z: %f.\n", V_aruco[0], V_aruco[1], V_aruco[2]);
			//printf("Aruco: Q - X: %f, Y: %f, Z: %f, W: %f.\n", Q_aruco[0], Q_aruco[1], Q_aruco[2], Q_aruco[3]);
			////if(arucoRPY[0] < 0.0) arucoRPY[0] = -3.14159-arucoRPY[0];
			////else if(arucoRPY[0] > 0.0) arucoRPY[0] = 3.14159-arucoRPY[0];
			//printf("Aruco RPY - Roll: %f / %f.\n", arucoRPY[0], arucoRPY[0]*(180/3.14159));
			//printf("Aruco RPY - Pitch: %f / %f.\n", arucoRPY[1], arucoRPY[1]*(180/3.14159));
			//printf("Aruco RPY - Yaw: %f / %f.\n", arucoRPY[2], arucoRPY[2]*(180/3.14159));

			//TODO: Once detection method for stabiliser raising has been physically implemented - add check here before allowing control.
			LT_effort = 0; RT_effort = 0;
			int track_dir = 0;
			//Aruco code x axis is the lateral distance when relative to the camera frame(V_aruco[0]).
 			//TODO: test flip case (right facing surface).
			if(_surface_direction == "left") track_dir = 1;
			else if(_surface_direction == "right") track_dir = -1; //flip direction for auto-align

			if(V_aruco[0] > T_aligned){
				_arucoAligned = false;
				LT_effort = 3000*track_dir;
				RT_effort = 3000*track_dir;
			}
			else if(V_aruco[0] < -T_aligned){
				_arucoAligned = false;
				LT_effort = -3000*track_dir;
				RT_effort = -3000*track_dir;
			}//end of translation alignment
			else _arucoAligned = true;

			if(_arucoAligned){
				arucoRPY[0] = arucoRPY[0]*(180/3.14159); //convert to deg for meaningful tolerance definition.
				if(arucoRPY[0] < -T_parallel){
					_arucoParallel = false;
					LT_effort = 1500*track_dir;
					RT_effort = -1500*track_dir;
				}
				else if(arucoRPY[0] > T_parallel){
					_arucoParallel = false;
					RT_effort = 1500*track_dir;
					LT_effort = -1500*track_dir;
				}
				else _arucoParallel = true;
			}//end of angle alignment

			if (_arucoAligned && _arucoParallel) printf("Target code alignment within tolerance. [Te, Ae] = [%f, %f].\n", V_aruco[0], arucoRPY[0]);
			else{
				can_tx.addData(0x93, LT_effort);
				can_tx.addData(0x94, RT_effort);
				printf("Translation error: %f meters, orientation error: %f degrees.\n", V_aruco[0], arucoRPY[0]);
				printf("Left track effort: %d, right track effort: %d.\n", LT_effort, RT_effort);
			}//end of else to apply track effort commands for alignment.
		}//end of else for case where target code is visible - autonomously align to wall/target code.
	}//end of autonomousTracks function

}//end of hinowacpp namespace

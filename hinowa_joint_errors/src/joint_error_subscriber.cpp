#include <hinowa_joint_errors/joint_error_subscriber.h>
#include <sensor_msgs/JointState.h>

namespace hinowa_joint_errors
{

	JOINT_ERROR_SUBSCRIBER::JOINT_ERROR_SUBSCRIBER()
	{
	}

	JOINT_ERROR_SUBSCRIBER::JOINT_ERROR_SUBSCRIBER(ros::NodeHandle nh)
	{
		this->_nh = nh;
	}

	JOINT_ERROR_SUBSCRIBER::~JOINT_ERROR_SUBSCRIBER()
	{
	}

	void JOINT_ERROR_SUBSCRIBER::init(std::vector<std::string> joint_names)
	{
		for(int i = 0; i < joint_names.size(); i++){
			joint_errors.push_back(Joint());
			joint_errors.at(i).name = joint_names.at(i);
		}
		
	}

	void JOINT_ERROR_SUBSCRIBER::storeData(const control_msgs::JointTrajectoryControllerState::ConstPtr& joint_states)
	{
		for(int i = 0; i < joint_errors.size(); i++){
			joint_errors.at(i).position.desired = joint_states->desired.positions[i];
			joint_errors.at(i).position.actual = joint_states->actual.positions[i];
			joint_errors.at(i).velocity.desired = joint_states->desired.velocities[i];
			joint_errors.at(i).velocity.actual = joint_states->actual.velocities[i];
		}	
	}


	void JOINT_ERROR_SUBSCRIBER::subscribe(ros::Duration dt, int SOT, int control_state)	
	{
		if(SOT == 1 && _prevSOT != 1){ //if trajectory is being executed on this loop, but wasn't on the previous loop..
			for(int i = 0; i < joint_errors.size(); i++){
				joint_errors.at(i).position.i_error = 0.0; //set accumulative error to 0.0
				joint_errors.at(i).velocity.i_error = 0.0; //set accumulative error to 0.0
			}
		}
		else if(SOT == 1 && _prevSOT == 1){
			this->_sub = _nh.subscribe<control_msgs::JointTrajectoryControllerState>("/hinowa/joint_trajectory_controller/state", 1000, &JOINT_ERROR_SUBSCRIBER::storeData, this);
			
			for(int i = 0; i < joint_errors.size(); i++){
//Position
				joint_errors.at(i).position.p_error = joint_errors.at(i).position.desired - joint_errors.at(i).position.actual;
				joint_errors.at(i).position.i_error += (joint_errors.at(i).position.p_error)*dt.toSec();
				joint_errors.at(i).position.d_error = (joint_errors.at(i).position.p_error-joint_errors.at(i).position.prev_p_error)/dt.toSec();

//velocity
				joint_errors.at(i).velocity.p_error = joint_errors.at(i).velocity.desired - joint_errors.at(i).velocity.actual;
				joint_errors.at(i).velocity.i_error += (joint_errors.at(i).velocity.p_error)*dt.toSec();
				joint_errors.at(i).velocity.d_error = (joint_errors.at(i).velocity.p_error-joint_errors.at(i).position.prev_p_error)/dt.toSec();


				joint_errors.at(i).velocity.prev_p_error = joint_errors.at(i).velocity.p_error;
				joint_errors.at(i).position.prev_p_error = joint_errors.at(i).position.p_error;
			}
			for(int i = 0; i < joint_errors.size(); i++){
				joint_position_errors[i] = joint_errors.at(i).position.p_error;
			}

		}
		_prevSOT = SOT;	

/* example code for what the above indexed code should do if not functioning as expected.
//Joint 1
			_J1AccumulativeError += _J1PError; 
			_J1IError = _J1AccumulativeError*dt.toSec();
			_J1DError = (_J1PError-_J1PErrorPrev)/dt.toSec();

		_J1PErrorPrev = _J1PError;

		printf("JOINT 1 ERRORS - P: %lf, I: %lf, D: %lf.\n", _J1PError, _J1IError, _J1DError);
*/
			
	}
}

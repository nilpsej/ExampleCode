#include <hinowa_hardware_interface/hinowa_hardware_interface.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace hinowa_hardware_interface
{
    hinowaHardwareInterface::hinowaHardwareInterface(ros::NodeHandle& nh, hinowacpp::HINOWA& hinowa, hinowacpp::REMOTE& remote, hinowa_joint_errors::JOINT_ERROR_SUBSCRIBER& joint_error_subscriber, hinowacpp::LEVEL_SENSOR& level_sensor, hinowacpp::STATUS_SUBSCRIBER& trajectory_status, camera_tf2::ARUCO_SUBSCRIBER& aruco_sub, hinowacpp::CAN_RX_SUBSCRIBER& can_rx, hinowacpp::CAN_TX_PUBLISHER& can_tx, hinowacpp::encoder_filter& filter, hinowa_parameters::hwi_publisher& HWI_pub, hinowa_parameters::ctrl_loop_subscriber& control_loop_sub, hinowa_joint_errors::JOINT_ERROR_PUBLISHER& joint_error_publisher) : nh_(nh), hinowa_(hinowa), remote_(remote), joint_error_subscriber_(joint_error_subscriber), level_sensor_(level_sensor), trajectory_status_(trajectory_status), aruco_sub_(aruco_sub) , can_rx_(can_rx), can_tx_(can_tx), filter_(filter), HWI_pub_(HWI_pub), control_loop_sub_(control_loop_sub), joint_error_publisher_(joint_error_publisher){        
	init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("/hinowa/hardware_interface/loop_hz", loop_hz_, 0.1);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &hinowaHardwareInterface::update, this);
    }

    hinowaHardwareInterface::~hinowaHardwareInterface() {

    }

    void hinowaHardwareInterface::init() {
        // Get joint names
        nh_.getParam("/hinowa/hardware_interface/joints", joint_names_);
	boost::shared_ptr<urdf::ModelInterface> urdf;
        num_joints_ = joint_names_.size();
	//printf("number of joints: %d.\n", num_joints_);
	joint_error_subscriber_.init(joint_names_);

        // Resize vectors
        joint_position_.resize(num_joints_);
	prev_joint_position_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
	prev_joint_velocity_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_acceleration_.resize(num_joints_);
	//joint_acceleration_command_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        //joint_effort_command_.resize(num_joints_);

        // Initialize Controller 
        for (int i = 0; i < num_joints_; ++i) {
		hinowacpp::Joint joint = hinowa_.getJoint(joint_names_[i]);
		prev_joint_position_[i] = hinowa_.getJoint(joint_names_[i]).readAngle(can_rx_, false);

		JointLimits limits;
		SoftJointLimits softLimits;
		//urdf::JointConstSharedPtr urdf_joint = urdf->getJoint(joint_names_[i]);
		//getJointLimits(urdf_joint, limits);
		getJointLimits(joint_names_[i], nh_, limits);

		// Create joint state interface
		JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
		joint_state_interface_.registerHandle(jointStateHandle);
/*
		// Create position joint interface
		JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
		position_joint_interface_.registerHandle(jointPositionHandle);
		PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
		positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);

		// Create effort joint interface
		JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
		effort_joint_interface_.registerHandle(jointEffortHandle);

		// Create velocity joint interface
		JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
		velocity_joint_interface_.registerHandle(jointVelocityHandle);

		// Create pos/vel/acc joint interface
		PosVelAccJointHandle PVAJointHandle(jointStateHandle, &joint_position_command_[i], &joint_velocity_command_[i], &joint_acceleration_command_[i]);
		PVA_joint_interface_.registerHandle(PVAJointHandle);
*/
		// Create pos&vel joint interface
		PosVelJointHandle PVJointHandle(jointStateHandle, &joint_position_command_[i], &joint_velocity_command_[i]);
		PV_joint_interface_.registerHandle(PVJointHandle);

        }

	registerInterface(&joint_state_interface_);
	//registerInterface(&position_joint_interface_);
	//registerInterface(&effort_joint_interface_);
	//registerInterface(&velocity_joint_interface_);
	//registerInterface(&PVA_joint_interface_);
	registerInterface(&PV_joint_interface_);
	registerInterface(&positionJointSoftLimitsInterface);

    }

    void hinowaHardwareInterface::update(const ros::TimerEvent& e) {

	SOT_ = trajectory_status_.subscribe();
	control_loop_sub_.subscribe();
	control_state_ = control_loop_sub_.state; //change this so that all "control_state_" mentions are replaced with control_loop_sub_.state
	active_valve_block_ = control_loop_sub_.active_valve_block;
//std::cout<<"Control loop params - state: "<<control_loop_sub_.state<<", valve block: "<<control_loop_sub_.active_valve_block<<", ball valve: "<<control_loop_sub_.ball_valve<<", reset_levelling: "<<control_loop_sub_.reset_levelling<<".\n";

	//printf("Control state: %d.\n", control_state_);
	if(active_valve_block_ == "bottom"){
		//printf("Bottom valves.\n");	
		can_tx_.addData(0x198, 1); //give heartbeat for bottom block
		can_tx_.addData(0x200, 0); //Bottom valve required high (12V) for output pin 46 on bot controller.
	}
	else if(active_valve_block_ == "top"){
	//	printf("Top valves.\n");
		can_tx_.addData(0x200, 1); //top valve requires low (0V) output for pin 46 on bot controller.
			if(SOT_ == 1){ //if trajectory currently being executed, allow valve actuation.
				can_tx_.addData(0x199, 1);//heartbeat message for top controller (status = 1 means in progress). 
			}
		
	}
	else printf("Both valve blocks locked.\n");

        elapsed_time_ = ros::Duration(e.current_real - e.last_real);

	HWI_pub_.addData(level_sensor_.is_level, SOT_, 1/elapsed_time_.toSec()); //levelled, state_of_trajectory, loop_hz
	HWI_pub_.publish();

        read(elapsed_time_);
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
	//printf("Elapsed time HWI loop: %f (%fHz).\n", elapsed_time_.toSec(), 1/elapsed_time_.toSec());
    }

    void hinowaHardwareInterface::read(ros::Duration elapsed_time) {
	can_rx_.subscribe();

        for (int i = 0; i < num_joints_; i++) {
//TODO: test whether this filter improves or hinders performance - do other values help?
/*		
		if(i == 1 || i == 2 || i == 4) {
			//std::cout<<"filtered "<<joint_names_[i]<<": ";
			joint_position_[i] = filter_.angle(joint_position_[i], i);
		}
*/
           	joint_position_[i] = hinowa_.getJoint(joint_names_[i]).readAngle(can_rx_, false);
		joint_velocity_[i] = (joint_position_[i]-prev_joint_position_[i])/elapsed_time.toSec();
		joint_acceleration_[i] = (joint_velocity_[i]-prev_joint_velocity_[i])/elapsed_time.toSec();
		prev_joint_position_[i] = joint_position_[i];
		prev_joint_velocity_[i] = joint_velocity_[i];
        }
	
	remote_.read(); //read remote regardless of operation state.

	if(control_state_ == 1 || control_state_ == 7) level_sensor_.read(can_rx_); //should always be 1 & last state


// aruco code pose monitoring, used during state 0 (for driving parallel to wall).
	
	if(control_state_ == 0){
		aruco_sub_output = aruco_sub_.subscribe();
		remote_.updateArucoFeedback(aruco_sub_output, control_loop_sub_.target_aruco_ID);

	}
    }

    void hinowaHardwareInterface::write(ros::Duration elapsed_time) {
        positionJointSoftLimitsInterface.enforceLimits(elapsed_time);

	joint_error_subscriber_.subscribe(elapsed_time, SOT_, control_state_);
	joint_error_publisher_.addErrorTerms(joint_error_subscriber_.joint_errors, joint_acceleration_, joint_velocity_command_);
	remote_.write(can_tx_, control_state_, control_loop_sub_.ball_valve);

	if((control_state_ == 1)) level_sensor_.level(nh_, can_tx_, control_loop_sub_.reset_levelling);
	else if(control_state_ == 7) level_sensor_.unlevel(nh_, can_tx_); //should always be last state

	//printf("state equals: %d\n", control_state_);
	if((active_valve_block_ == "top") && (SOT_ == 1)){
		printf("//--//\n");
        	for (int i = 0; i < num_joints_; i++) {
			hinowa_.getJoint(joint_names_[i]).actuate(joint_velocity_command_[i], can_tx_, joint_error_subscriber_.joint_position_errors[i], 1);
			//hinowa_.getJoint(joint_names_[i]).actuate(joint_velocity_command_[i],can_tx_, 1);
			//TEST THIS TO STOP WOBBLE - seems workable but could be better to lower threshold on joint 1 deadzone
			if((/*control_state_ == 5 ||*/ control_state_ == 4 || control_state_ == 3) && i == 0)
				hinowa_.getJoint(joint_names_[i]).actuate(0.0,can_tx_,joint_error_subscriber_.joint_position_errors[i], 1);
/*
			else hinowa_.getJoint(joint_names_[i]).actuate(joint_velocity_command_[i], can_tx_, joint_error_subscriber_.joint_position_errors[i], 1);
*/
        	}
	}
	else if((active_valve_block_ == "top") && (SOT_ != 1)){
		for (int i = 0; i < num_joints_; i++) {
			hinowa_.getJoint(joint_names_[i]).actuate(0.0,can_tx_,joint_error_subscriber_.joint_position_errors[i],0);
		}
	}
	joint_error_publisher_.publish();
	can_tx_.publish(); //publish should be called ONCE at the end of the write function.
	ros::spinOnce(); //check usefulness
    }
}


#include<hinowa_control/control_subscriber.h>
#include<hinowa_control/plan_result_subscriber.h>
#include<hinowa_control/execute_result_subscriber.h>

#include <fstream>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <iostream>
//#include <moveit/robot_model/robot_model.h>
//#include <moveit/robot_state/robot_state.h>
//#include <ros/ros.h>
//#include <ros/console.h>
//#include <nodelet/nodelet.h>
//#include <std_msgs/Header.h>
//#include <std_msgs/Bool.h>
//#include "std_msgs/String.h"
//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>
//#include <tf/transform_datatypes.h>
/*
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <moveit/core/transforms/transforms.h>	// Eigen transforms
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Joy.h>
*/	
//#include<bits/stdc++.h>

using namespace std; 

std::tuple<int,int,int> controller_output;
int Select; int Start; int PS;
int homePrimed = 0;
int state = 0;
int SOT = 0;
const string stateInfo[] = {"Remote track and stabiliser control", "Autonomous levelling", "Rotate to face wall", "Aruco code detection", "Move to top of building", "Apply paint to surface", "Autonomous unlevelling"};//, "Painting result state"
bool primed = false;
double buildingHeight;
double distanceFromWall;
const string buildingDirection = "left";
geometry_msgs::Point EEFxyz_prev;
double t_prev = 0;
geometry_msgs::Pose camera;
geometry_msgs::Pose aruco;
//double r = 0.0; double p = 0.0; double y = 0.0;
double tp0 = 0.0;
double paintDelay = 0.0;
bool enablePainting = false;
moveit::planning_interface::MoveGroupInterface::Plan painting_plan;
int newExecutionResult;
int executionResult;
//-----------------------------------------------------------------------------------------------------------------------------------//
struct eulerAngles
{
    double roll, pitch, yaw;
};

void state0Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, const robot_state::JointModelGroup* joint_model_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//currently track control
	int length = sizeof(stateInfo)/sizeof(stateInfo[0]);
	if(action == -2){ //this state will have stabilisers in unpredictable positions - cannot allow return to home condition. 
	}	
	if(action == -1){ //not necessary without the above being functional.
	}
	else if(action == 0){//not necessary without the above being functional.
		cout<<"Current state: "<<stateInfo[length]<<". Next state: "<<stateInfo[0]<<", has been unprimed.\n";
	}
	else if(action == 1){// _nh.setParam("/control/activeValveBlock", "bottom");
		cout<<"Current state: "<<stateInfo[length]<<". Next state: "<<stateInfo[0]<<", has been primed.\n";

	}

	else if(action == 2){
		cout<<"Executing state: "<<stateInfo[1]<<".\n";
		state = 1;
		_nh.setParam("/control/activeValveBlock", "bottom");
		_nh.setParam("/control/levelled", 0);
		_nh.setParam("/level_sensor/firstSuccess", 1);
		_nh.setParam("/level_sensor/startingLevelling", 1);
	}
}
	
void state1Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, const robot_state::JointModelGroup* joint_model_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){ //auto level			
	if(action == -2){//this state will not be guaranteed to have the machine levelled - cannot allow return to home condition.
	}
	else if(action == -1){//not necessary without the above being functional.
	}	
	else if(action == 0){//not necessary without the above being functional.
		cout<<"Current state: "<<stateInfo[0]<<". Next state: "<<stateInfo[1]<<", has been unprimed.\n";
/*
		move_group.setPlannerId("PTP");
		//pointer to reference robots state
		moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
		// Next get the current set of joint values for the group.
		std::vector<double> joint_group_positions;
		current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
		move_group.setJointValueTarget(joint_group_positions);
		move_group.plan(global_plan);
*/
	}
	else if(action == 1){
		cout<<"Current state: "<<stateInfo[0]<<". Next state: "<<stateInfo[1]<<", has been primed.\n";
	}
	else if(action == 2){
		int levelled;		
		_nh.getParam("/control/levelled", levelled);
		if(levelled){ //UNCOMMENT THESE LINES FOR SAFETY WHEN OPERATING, COMMENT FOR VIRTUAL CAN TESTS
			cout<<"Executing state: "<<stateInfo[2]<<".\n";
			printf("State: %d.\n", state);
			state = 2;
			_nh.setParam("/control/activeValveBlock", "top");
			bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			printf(result ? "succeeded\n" : "failed\n");
		}
		else state = 1;
	}
	
}
void state2Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, const robot_state::JointModelGroup* joint_model_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//rotate 90 deg

	int levelled;		
	_nh.getParam("/control/levelled", levelled);

	if(action == -2){
		if(levelled){
			printf("EXECUTE TO HOME POSE.\n");
			_nh.setParam("/control/activeValveBlock", "top");
			bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			printf(result ? "succeeded\n" : "failed\n");
			state = 0;
		}
		else{
			printf("Machine not level - cannot return to home position safely.\n");
			state = 1;
		}
	}
	else if(action == -1){
		if(levelled){ 
			//have a try catch statement here for PTP, LIN or CIRC planning to ensure the planning succeeds
			printf("PLAN TO HOME POSE.\n");
			move_group.setPlannerId("PTP");
			move_group.setNamedTarget("home");
			move_group.plan(global_plan);
		}
		else{
			printf("Machine not level - cannot return to home position safely.\n");
			state = 1;
		}
	}	
	else if(action == 0){
		cout<<"Current state: "<<stateInfo[1]<<". Next state: "<<stateInfo[2]<<", has been unprimed.\n";
		move_group.setPlannerId("PTP");
		//pointer to reference robots state
		moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
		// Next get the current set of joint values for the group.
		std::vector<double> joint_group_positions;
		current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
		move_group.setJointValueTarget(joint_group_positions);
		move_group.plan(global_plan);
	}
	else if(action == 1){
		cout<<"Current state: "<<stateInfo[1]<<". Next state: "<<stateInfo[2]<<", has been primed.\n";
		move_group.setPlannerId("CIRC");
		
		//PATH CONSTRAINTS
		move_group.setMaxAccelerationScalingFactor(0.2);
		move_group.setMaxVelocityScalingFactor(0.2);
		move_group.setGoalTolerance(0.1);

		moveit_msgs::PositionConstraint interim_point;
		interim_point.link_name = "end";
		interim_point.weight = 1.0;
		interim_point.header.frame_id = "anchor";
		
		interim_point.constraint_region.primitive_poses.resize(1);
		if(buildingDirection == "left"){
			interim_point.constraint_region.primitive_poses[0].position.x = -2.0;
			interim_point.constraint_region.primitive_poses[0].position.y = 1.75;
			interim_point.constraint_region.primitive_poses[0].position.z = 1.5;
		}
		else if(buildingDirection == "right"){
			interim_point.constraint_region.primitive_poses[0].position.x = 2.0;
			interim_point.constraint_region.primitive_poses[0].position.y = 1.75;
			interim_point.constraint_region.primitive_poses[0].position.z = 1.5;
		}

		moveit_msgs::Constraints interim_constraint;
		interim_constraint.name = "interim";
		interim_constraint.position_constraints.push_back(interim_point);

		move_group.setPathConstraints(interim_constraint);
//----//
		//positive z is upward in world frame, positive y is to the left(?) in world frame from start view
		geometry_msgs::Pose pose = move_group.getCurrentPose().pose;
		if(buildingDirection == "left"){
			pose.position.x = -2.42043; //figure out these values using the lift with real CAN
			pose.position.y = 0.0; 
			pose.position.z = 0.912296;
			pose.orientation.x = 0.0;
			pose.orientation.y = 0.0;
			pose.orientation.z = 0.7071;
			pose.orientation.w = 0.7071;
		}
		else if(buildingDirection == "right"){
			pose.position.x = 2.42043; //figure out these values using the lift with real CAN
			pose.position.y = 0.0; 
			pose.position.z = 0.912296;
			pose.orientation.x = 0.0;
			pose.orientation.y = 0.0;
			pose.orientation.z = -0.7071;
			pose.orientation.w = 0.7071;
		}
		move_group.setPoseTarget(pose);
		//move_group.setPlanningTime (30.0);
		
		bool result = (move_group.plan(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("INFO","Visualizing plan 1 (pose goal) %s", result ? "succeeded" : "FAILED");

//printing out trajectory plan
		moveit_msgs::RobotTrajectory trajectory = global_plan.trajectory_;
		std::ofstream outfile("/home/josh/workspace/src/hinowacpp/points.csv",std::ios::app);
         	//std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajectory_points1;  
		std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = trajectory.joint_trajectory.points; 

         	std::vector<int>::size_type size = trajectory_points.size(); 
		std::vector<int>::size_type numJoints = trajectory.joint_trajectory.joint_names.size();
		
		for(unsigned i = 0; i<size; i++){
			outfile<<"ROTATE TO FACE WALL - point_index: "<<i<<"\n";
			for (unsigned j=0; j<numJoints; j++){
				outfile<<trajectory.joint_trajectory.joint_names[j]<<","<<trajectory_points[i].positions[j]<<","<<trajectory_points[i].velocities[j]<<","<<trajectory_points[i].accelerations[j]<<",";
			}
			outfile<<"\n";
		}
		outfile.close();
	}
	else if(action == 2){
		int levelled;		
		_nh.getParam("/control/levelled", levelled);
		//if(levelled){ //UNCOMMENT THESE LINES FOR SAFETY WHEN OPERATING, COMMENT FOR VIRTUAL CAN TESTS
			cout<<"Executing state: "<<stateInfo[2]<<".\n";
			printf("State: %d.\n", state);
			//state = 3;
			_nh.setParam("/control/activeValveBlock", "top");
			bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			printf(result ? "succeeded\n" : "failed\n");
		//}
		//else state = 1;
		
	}
}
void state3Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, const robot_state::JointModelGroup* joint_model_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//Aruco code detection
	if(action == -2){
		_nh.setParam("/control/activeValveBlock", "top");
		bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		printf(result ? "succeeded\n" : "failed\n");
		state = 0;
	}
	else if(action == -1){
		move_group.setPlannerId("PTP");
		move_group.setNamedTarget("home");
		move_group.plan(global_plan);
	}
	else if(action == 0){
		cout<<"Current state: "<<stateInfo[2]<<". Next state: "<<stateInfo[3]<<", has been unprimed.\n";
		move_group.setPlannerId("PTP");
		//pointer to reference robots state
		moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
		// Next get the current set of joint values for the group.
		std::vector<double> joint_group_positions;
		current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
		move_group.setJointValueTarget(joint_group_positions);
		move_group.plan(global_plan);
	}
	else if(action == 1){
		cout<<"Current state: "<<stateInfo[2]<<". Next state: "<<stateInfo[3]<<", has been primed.\n";
	}
	else if(action == 2){
		cout<<"Executing state: "<<stateInfo[4]<<".\n";
		state = 4;
	}
	
}
void state4Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, const robot_state::JointModelGroup* joint_model_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//move to top position
	if(action == -2){
		_nh.setParam("/control/activeValveBlock", "top");
		bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		printf(result ? "succeeded\n" : "failed\n");
		state = 0;
	}
	else if(action == -1){
		move_group.setPlannerId("PTP");
		move_group.setNamedTarget("home");
		move_group.plan(global_plan);
	}
	else if(action == 0){
		move_group.setPlannerId("PTP");
		//pointer to reference robots state
		moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
		// Next get the current set of joint values for the group.
		std::vector<double> joint_group_positions;
		current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
		move_group.setJointValueTarget(joint_group_positions);
		move_group.plan(global_plan);
	}
	else if(action == 1){
		move_group.setPlannerId("LIN");
//SCALING FACTORS
		move_group.setMaxAccelerationScalingFactor(0.3);
		move_group.setMaxVelocityScalingFactor(0.3);
		move_group.setGoalTolerance(0.1);
//---//
		//positive z is upward in world frame, positive y is to the left(?) in world frame from start view
		geometry_msgs::Pose pose = move_group.getCurrentPose().pose;

		if(buildingDirection == "left"){
			pose.position.x = -3; //3.2, eventually will be = camera sensed distance from aruco code
			pose.position.y = 0.0; 
			pose.position.z = 4.7; //eventually will be = buildingHeight
			pose.orientation.x = 0.0;
			pose.orientation.y = 0.0;
			pose.orientation.z = 0.7071;
			pose.orientation.w = 0.7071;
		}
		else if(buildingDirection == "right"){
			pose.position.x = 3.1; //3.2, eventually will be = camera sensed distance from aruco code
			pose.position.y = 0.0; 
			pose.position.z = 4.7; //eventually will be = buildingHeight
			pose.orientation.x = 0.0;
			pose.orientation.y = 0.0;
			pose.orientation.z = -0.7071;
			pose.orientation.w = 0.7071;
		}

		move_group.setPoseTarget(pose);
		//move_group.setPlanningTime (30.0);
		
		bool result = (move_group.plan(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("INFO","Visualizing plan 1 (pose goal): %s", result ? "succeeded" : "FAILED");

//printing out trajectory plan
		moveit_msgs::RobotTrajectory trajectory = global_plan.trajectory_;
		std::ofstream outfile("/home/josh/workspace/src/hinowacpp/points.csv",std::ios::app);
         	//std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajectory_points1;  
		std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = trajectory.joint_trajectory.points; 

         	std::vector<int>::size_type size = trajectory_points.size(); 
		std::vector<int>::size_type numJoints = trajectory.joint_trajectory.joint_names.size();
		
		for(unsigned i = 0; i<size; i++){
			outfile<<"MOVE TO TOP OF BUILDING - point_index: "<<i<<"\n";
			for (unsigned j=0; j<numJoints; j++){
				outfile<<trajectory.joint_trajectory.joint_names[j]<<","<<trajectory_points[i].positions[j]<<","<<trajectory_points[i].velocities[j]<<","<<trajectory_points[i].accelerations[j]<<",";
			}
			outfile<<"\n";
		}
		outfile.close();
//------//

	}
	else if(action == 2){
		_nh.setParam("/control/activeValveBlock", "top");
		move_group.asyncExecute(global_plan);
	}
}

void state5Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, const robot_state::JointModelGroup* joint_model_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//paint..
	if(action == -2){
		_nh.setParam("/control/activeValveBlock", "top");
		bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		printf(result ? "succeeded\n" : "failed\n");
		state = 0;
	}
	else if(action == -1){
		move_group.setPlannerId("PTP");
		move_group.setNamedTarget("home");
		move_group.plan(global_plan);
	}
	else if(action == 0){
		move_group.setPlannerId("PTP");
		//pointer to reference robots state
		moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
		// Next get the current set of joint values for the group.
		std::vector<double> joint_group_positions;
		current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
		move_group.setJointValueTarget(joint_group_positions);
		move_group.plan(global_plan);
	}
	else if(action == 1){
//SCALING FACTORS
		move_group.setPlannerId("LIN");
		move_group.setMaxAccelerationScalingFactor(0.1); //test these numbers
		move_group.setMaxVelocityScalingFactor(0.05); //test these numbers
		move_group.setGoalTolerance(0.1);
//---//

//JOINT 1 CONSTRAINT -----------------------> WORK ON THIS WITH FEEDBACK OF ARUCO CODE IMPLEMENTATION AND ACCOUNTING FOR PARALLEL SKEW

//this isn't currently working - there is still small effort values being sent to joint_1. Try reworking this, or having a catch statement for valve actuation that says if the current state is = 5, do not allow actuation of joint_1.
/* 
		moveit_msgs::JointConstraint lock_j1;
		lock_j1.joint_name = "joint_1";
		lock_j1.tolerance_above = 0.0;
		lock_j1.tolerance_below = 0.0;
		if(buildingDirection == "left") lock_j1.position = 1.571; //figure out how to determine these values - should they read current
		else if(buildingDirection == "right") lock_j1.position = -1.571; //j1 value and assign that...? investigate best solution.
		lock_j1.weight = 1.0;

		moveit_msgs::Constraints joint_constraint;
		joint_constraint.name = "Lock J1";
		joint_constraint.joint_constraints.push_back(lock_j1);

		move_group.setPathConstraints(joint_constraint);
*/
//---//


		//plan cartesian movement
		geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose; //record start pose (top position).
		std::vector<geometry_msgs::Pose> waypoints;
		//waypoints.push_back(start_pose); //add to waypoint list

		geometry_msgs::Pose final_pose = start_pose; //create copy of start_pose.
		//make changes required to form desired final pose.
		//final_pose.position.x -= 0.05;
		final_pose.position.z = 1.5; //1.1, set desired final z to 1m.
		waypoints.push_back(final_pose);

		geometry_msgs::Pose retract_pose = final_pose;

		if(buildingDirection == "left") retract_pose.position.x = -2.42043;
		else if(buildingDirection == "right") retract_pose.position.x = 2.42043;
		retract_pose.position.y = 0.0;
		retract_pose.position.z = 0.912296;
		waypoints.push_back(retract_pose);

		moveit_msgs::RobotTrajectory trajectory;
		
		const double jump_threshold = 100; //was 1000
 		const double eef_step = 0.05;
		const bool collisions = true;

		double success_fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, collisions);
		ROS_INFO_NAMED("INFO", "Visualizing plan for painting (Cartesian path) (%.2f%% achieved)", success_fraction * 100.0);
		
		// The trajectory needs to be modified so it will include velocities as well.
  		// First to create a RobotTrajectory object
  		robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "all");

  		// Second get a RobotTrajectory from trajectory
  		rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

		// Thrid create a IterativeParabolicTimeParameterization object
  		trajectory_processing::IterativeParabolicTimeParameterization iptp;

  		// Fourth compute computeTimeStamps
  		bool success = iptp.computeTimeStamps(rt);
  		ROS_INFO("Computed time stamp %s",success?"SUCCEEDED":"FAILED");

  		// Get RobotTrajectory_msg from RobotTrajectory
  		rt.getRobotTrajectoryMsg(trajectory);

  		global_plan.trajectory_ = trajectory;

//printing out trajectory plan
		//moveit_msgs::RobotTrajectory trajectory = global_plan.trajectory_;
		std::ofstream outfile("/home/josh/workspace/src/hinowacpp/points.csv",std::ios::app);
         	//std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajectory_points1;  
		std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = trajectory.joint_trajectory.points; 

         	std::vector<int>::size_type size = trajectory_points.size(); 
		std::vector<int>::size_type numJoints = trajectory.joint_trajectory.joint_names.size();
		for(unsigned i = 0; i<size; i++){
			outfile<<"PAINTING MOVEMENT - point_index: "<<i<<"\n";
			for (unsigned j=0; j<numJoints; j++){
				outfile<<trajectory.joint_trajectory.joint_names[j]<<","<<trajectory_points[i].positions[j]<<","<<trajectory_points[i].velocities[j]<<","<<trajectory_points[i].accelerations[j]<<",";
			}
			outfile<<"\n";
		}
		outfile.close();
//------//
	}
	else if(action == 2){
		_nh.setParam("/control/activeValveBlock", "top");
		tp0 = ros::Time::now().toSec();
		enablePainting = true;
		painting_plan = global_plan;
		_nh.setParam("/control/ballValve", 1);
		printf("Valve open.\n");
/*
		//execute cartesian movement
		moveit::planning_interface::MoveItErrorCode execute_outcome;  	
		execute_outcome = move_group.asyncExecute(global_plan); //uncomment to execute
*/
	}
}

void performPainting(moveit::planning_interface::MoveGroupInterface& move_group){
	double t_now = ros::Time::now().toSec();
	if((t_now - tp0) > 2.0){
		//execute cartesian movement
		moveit::planning_interface::MoveItErrorCode execute_outcome;  	
		execute_outcome = move_group.asyncExecute(painting_plan); //uncomment to execute	
		enablePainting = false;
	}
}

void state6Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, const robot_state::JointModelGroup* joint_model_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//post paint state, unlevel+raise stabilisers
	if(action == -2){
		_nh.setParam("/control/activeValveBlock", "top");
		bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		printf(result ? "succeeded\n" : "failed\n");
		state = 0;
	}
	else if(action == -1){
		move_group.setPlannerId("PTP");
		move_group.setNamedTarget("home");
		move_group.plan(global_plan);
	}
	else if(action == 0){
/* dont know if this is needed before testing. 
		move_group.setPlannerId("PTP");
		//pointer to reference robots state
		moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
		// Next get the current set of joint values for the group.
		std::vector<double> joint_group_positions;
		current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
		move_group.setJointValueTarget(joint_group_positions);
		move_group.plan(global_plan);
*/
	}
	else if(action == 1){
		//nothing performed here currently, should probably have checks to see if enabling destabilising is safe ie. joint positions.
	}
	else if(action == 2){
		_nh.setParam("/control/activeValveBlock", "bottom");
		//Apply actions for raising stabilisers back off the ground - monitor the gyro while doing so to inform effort values. Once all lights are off, have a time measured application of full effort to raise them a reasonable amount to allow driving without collision.
	}
}

//-----------------------------------------------------------------------------------------------------------------------------------//

void stateUpdater(ros::NodeHandle _nh, control::EXECUTE_RESULT_SUBSCRIBER execute_subscriber, int executionResult){
	printf("Execution result: %d.\n",executionResult);
	//execute_subscriber.subscribe();
/*
	int length = sizeof(stateInfo)/sizeof(stateInfo[0]);
	int nextState = state+1;
	if((nextState) >= length) nextState = 0;
*/
 
/*
	bool virtualCan;
	_nh.getParam("/CAN/virtual", virtualCan);
	if(virtualCan){ //can is virtual, testing without proper execution, do not consider any execution failure/feedback for state advancement
		
		if(executionType == "advance") state = nextState;
		else if(executionType == "home") state = 0;

	} 
	else{
	}
*/
}


void stateHandler(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, const robot_state::JointModelGroup* joint_model_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan)
{
//move this into a new state advancement function
		//printf("Select: %d, start: %d\n.", Select, Start);
//		int length = sizeof(stateInfo)/sizeof(stateInfo[0]);
//		int nextState = state+1;
//		if((nextState) >= length) nextState = 0; 

		if(Select && !Start){//planning.
			homePrimed = false;
			if(!primed) primed = true;
			else primed = false;
		
			if(!primed){				
				switch(state){//all action functions should be case+1 because these are planning cases (visualise/prep the next state).
				case 0 : state1Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				case 1 : state2Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				case 2 : state3Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				case 3 : state4Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				case 4 : state5Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				case 5 : state6Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				case 6 : state0Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				}//end of !primed switch.
			}//end of !primed if.
			else if(primed){
				switch(state){
				case 0 : state1Action(_nh, move_group, joint_model_group, global_plan, 1);
					 break;
				case 1 : state2Action(_nh, move_group, joint_model_group, global_plan, 1);
					 break;
				case 2 : state3Action(_nh, move_group, joint_model_group, global_plan, 1);
					 break;
				case 3 : state4Action(_nh, move_group, joint_model_group, global_plan, 1);
					 break;
				case 4 : state5Action(_nh, move_group, joint_model_group, global_plan, 1);
					 break;
				case 5 : state6Action(_nh, move_group, joint_model_group, global_plan, 1);
					 break;
				case 6 : state0Action(_nh, move_group, joint_model_group, global_plan, 1);
					 break;
				}//end of primed switch.
			}//end of primed if.
//			cout<<"Current state: "<<stateInfo[state]<<". Next state: "<<stateInfo[nextState]<<", has been ";
//			printf(primed ? "primed.\n" : "unprimed.\n");
		}//end of planning conditional
		else if(PS && !Select && !Start){//planning for return to home condition for all states.
			primed = false; 
			if(!homePrimed) homePrimed = true;
			else homePrimed = false;
			if(!homePrimed){
				switch(state){//all action functions should be case+1 because these are planning cases (visualise/prep the next state).
				case 0 : state1Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				case 1 : state2Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				case 2 : state3Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				case 3 : state4Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				case 4 : state5Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				case 5 : state6Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				case 6 : state0Action(_nh, move_group, joint_model_group, global_plan, 0);
					 break;
				}//end of !homePrimed switch.
			}
			else if(homePrimed){
				switch(state){
				case 0 : state1Action(_nh, move_group, joint_model_group, global_plan, -1);
					 break;
				case 1 : state2Action(_nh, move_group, joint_model_group, global_plan, -1);
					 break;
				case 2 : state3Action(_nh, move_group, joint_model_group, global_plan, -1);
					 break;
				case 3 : state4Action(_nh, move_group, joint_model_group, global_plan, -1);
					 break;
				case 4 : state5Action(_nh, move_group, joint_model_group, global_plan, -1);
					 break;
				case 5 : state6Action(_nh, move_group, joint_model_group, global_plan, -1);
					 break;
				case 6 : state0Action(_nh, move_group, joint_model_group, global_plan, -1);
					 break;
				}//end of homePrimed switch.
			}
		}

		else if(!Select && Start && primed && !homePrimed){//executing state advancement.
			primed = false;
			//state = nextState; //move this to new state advancement function. 
			//cout<<"Executing state: "<<stateInfo[state]<<".\n";

			switch(state){
			case 0 : state0Action(_nh, move_group, joint_model_group, global_plan, 2);
				 break;
			case 1 : state1Action(_nh, move_group, joint_model_group, global_plan, 2);
				 break;
			case 2 : state2Action(_nh, move_group, joint_model_group, global_plan, 2);
				 break;
			case 3 : state3Action(_nh, move_group, joint_model_group, global_plan, 2);
				 break;
			case 4 : state4Action(_nh, move_group, joint_model_group, global_plan, 2);
				 break;
			case 5 : state5Action(_nh, move_group, joint_model_group, global_plan, 2);
				 break;
			case 6 : state6Action(_nh, move_group, joint_model_group, global_plan, 2);
				 break;
			}//end of executing if.
		}//end of execution conditional	

		else if(!Select && Start && !primed && homePrimed){//executing return to home.
			homePrimed = false;
			//state = nextState; //move this to new state advancement function.
//			cout<<"Executing state: "<<stateInfo[state]<<".\n";

			switch(state){
			case 0 : state0Action(_nh, move_group, joint_model_group, global_plan, -2);
				 break;
			case 1 : state1Action(_nh, move_group, joint_model_group, global_plan, -2);
				 break;
			case 2 : state2Action(_nh, move_group, joint_model_group, global_plan, -2);
				 break;
			case 3 : state3Action(_nh, move_group, joint_model_group, global_plan, -2);
				 break;
			case 4 : state4Action(_nh, move_group, joint_model_group, global_plan, -2);
				 break;
			case 5 : state5Action(_nh, move_group, joint_model_group, global_plan, -2);
				 break;
			case 6 : state6Action(_nh, move_group, joint_model_group, global_plan, -2);
				 break;
			}//end of executing if.
		}//end of execution conditional

};//end of stateHandler

//-----------------------------------------------------------------------------------------------------------------------------------//

double EEFplotting(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, double t0){
		if(state == 5){
			_nh.getParam("/control/SOT", SOT);
			geometry_msgs::Point EEFxyz = move_group.getCurrentPose().pose.position;
			if(!enablePainting){			
				if((SOT == 1) && (EEFxyz.z > (1.5+1.3))){
					_nh.setParam("/control/ballValve", 1);
					printf("Valve open.\n");
				}
				else{
					_nh.setParam("/control/ballValve", 0);
					//printf("Valve closed. SOT: %d, EEF: %f.\n", SOT, EEFxyz.z);
				}
			}
			if((EEFxyz.z != EEFxyz_prev.z)){
				double t = ros::Time::now().toSec()-t0;
				//printf("End effector position, X: %f, Y: %f, Z: %f, t: %f.\n", EEFxyz.x, EEFxyz.y, EEFxyz.z, t);
				std::ofstream outfile("/home/josh/workspace/src/hinowacpp/EEFposition.csv",std::ios::app);
				outfile<<EEFxyz.x<<","<<EEFxyz.y<<","<<EEFxyz.z<<","<<t;
				//z velocity
				double Vz = (EEFxyz.z-EEFxyz_prev.z)/(t-t_prev);
				outfile<<","<<Vz;
	
				outfile<<"\n";
				outfile.close();
				EEFxyz_prev.x = EEFxyz.x; EEFxyz_prev.y = EEFxyz.y; EEFxyz_prev.z = EEFxyz.z; t_prev = t;

//add reference to a global variable/parameter that stores the camera sensed distance to aruco code/wall, such that the valve open command is only reached if the x value of the EEFpose is within a +- bound of some tolerance relative to the sensed distance. This is to ensure the sprayers dont turn back on when performing the retract-away-from-wall part of the movement added onto the end of each stripe. 

/*
				if((fabs(Vz) > 0.01) && (SOT == 1) && (EEFxyz.z > (1.5+1.3))){ //check this value of Vz comparison - experimentally
					_nh.setParam("/control/ballValve", 1);
					printf("Valve open.\n");
				}
				else{
					_nh.setParam("/control/ballValve", 0);
					printf("Valve closed.\n");
				}
*/
			}
			return t_prev;
			
		}
}

std::tuple<geometry_msgs::Pose, geometry_msgs::Pose> getArucoData(ros::NodeHandle _nh, geometry_msgs::Pose camera, geometry_msgs::Pose aruco){
	if(state == 0 || state == 3){//might have to add new yaml param for number of visible codes >0 t/f to add gaussian filter for these values (to avoid 0s overwriting desired values)..
//may have to add functionality for tracking a 2nd aruco code - one for origin and 1 for carrying of the origin between movements.
			double filterValue = 0.6; //set to 0 if no filtering needed/wanted. setting too high results in a lot of smoothing lag.
			geometry_msgs::Pose currentCamera; //geometry_msgs::Pose currentAruco; may need to filter 2nd aruco code when added
			_nh.getParam("/aruco_feedback/camera/position/x", currentCamera.position.x);
			_nh.getParam("/aruco_feedback/camera/position/y", currentCamera.position.y);
			_nh.getParam("/aruco_feedback/camera/position/z", currentCamera.position.z);
			_nh.getParam("/aruco_feedback/camera/orientation/x", currentCamera.orientation.x);
			_nh.getParam("/aruco_feedback/camera/orientation/y", currentCamera.orientation.y);
			_nh.getParam("/aruco_feedback/camera/orientation/z", currentCamera.orientation.z);
			_nh.getParam("/aruco_feedback/camera/orientation/w", currentCamera.orientation.w);

			_nh.getParam("/aruco_feedback/aruco_1/position/x", aruco.position.x);
			_nh.getParam("/aruco_feedback/aruco_1/position/y", aruco.position.y);
			_nh.getParam("/aruco_feedback/aruco_1/position/z", aruco.position.z);
			_nh.getParam("/aruco_feedback/aruco_1/orientation/x", aruco.orientation.x);
			_nh.getParam("/aruco_feedback/aruco_1/orientation/y", aruco.orientation.y);
			_nh.getParam("/aruco_feedback/aruco_1/orientation/z", aruco.orientation.z);
			_nh.getParam("/aruco_feedback/aruco_1/orientation/w", aruco.orientation.w);

			camera.position.x = (currentCamera.position.x*(1-filterValue))+(camera.position.x*filterValue);
			camera.position.y = (currentCamera.position.y*(1-filterValue))+(camera.position.y*filterValue);
			camera.position.z = (currentCamera.position.z*(1-filterValue))+(camera.position.z*filterValue);
			camera.orientation.x = (currentCamera.orientation.x*(1-filterValue))+(camera.orientation.x*filterValue);
			camera.orientation.y = (currentCamera.orientation.y*(1-filterValue))+(camera.orientation.y*filterValue);
			camera.orientation.z = (currentCamera.orientation.z*(1-filterValue))+(camera.orientation.z*filterValue);
			camera.orientation.w = (currentCamera.orientation.w*(1-filterValue))+(camera.orientation.w*filterValue);
			//printf("unfiltered x: %f.\n", currentCamera.position.x);

			//convert camera quarternion to euler angles, create yaml parameters for them (/aruco_feedback/camera_euler/r,p,y)
			//pull these three angles into remote.cpp file, use "z" angle to handle track effort values for PID & translational distance for the distance required for the tracks to travel to the next code (i.e if current code is id-1, id-2 will be the in-between code, move camera distance equal to translational displacement between it and id-2 until id-3 is visible, then update desired final translation to relative displacement between camera and id-3). 
			//x distance will be necessary to determine "move-to-top-of-building" and "painting-movement" positions for state 4/5. Will probably need to be filtered along with the rest of the values. 
		return std::make_tuple(camera, aruco);
	}
}

void q2e(ros::NodeHandle _nh, geometry_msgs::Pose camera){
//TEST AND REVISE THE LOGIC OF THE CONVERSION IN THIS FUNCTION ONCE CAMERA IS SETUP ON MACHINE.
	eulerAngles angles;
	
	double ysqr = camera.orientation.y * camera.orientation.y;
	double t0 = -2.0f * (ysqr + camera.orientation.z * camera.orientation.z) + 1.0f;
	double t1 = +2.0f * (camera.orientation.x * camera.orientation.x - camera.orientation.w * camera.orientation.z);
	double t2 = -2.0f * (camera.orientation.x * camera.orientation.z + camera.orientation.w * camera.orientation.y);
	double t3 = +2.0f * (camera.orientation.y * camera.orientation.z - camera.orientation.w * camera.orientation.x);
	double t4 = -2.0f * (camera.orientation.x * camera.orientation.x + ysqr) + 1.0f;

	t2 = t2 > 1.0f ? 1.0f : t2;
	t2 = t2 < -1.0f ? -1.0f : t2;

	angles.pitch = std::asin(t2);
	angles.roll = std::atan2(t3, t4);
	angles.yaw = std::atan2(t1, t0);

/*
	//roll (x-axis)
	double sinr_cosp = 2.0 * (camera.orientation.w * camera.orientation.x + camera.orientation.y * camera.orientation.z);
	double cosr_cosp = 1.0 - 2.0 * (camera.orientation.x * camera.orientation.x + camera.orientation.y * camera.orientation.y);
	angles.roll = atan2(sinr_cosp, cosr_cosp);

	//pitch (y-axis)
	double sinp = 2.0 * (camera.orientation.w * camera.orientation.y - camera.orientation.z * camera.orientation.x);
	if (fabs(sinp) >= 1) angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else angles.pitch = asin(sinp);

	//yaw (z-axis)
	double siny_cosp = 2.0 * (camera.orientation.w * camera.orientation.z + camera.orientation.x * camera.orientation.y);
	double cosy_cosp = 1.0 - 2.0 * (camera.orientation.y * camera.orientation.y + camera.orientation.z * camera.orientation.z);  
	angles.yaw = atan2(siny_cosp, cosy_cosp);
*/
	angles.roll = angles.roll*180.0/M_PI;	
	angles.pitch = angles.pitch*180.0/M_PI;
	angles.yaw = angles.yaw*180.0/M_PI;
	
	_nh.setParam("/aruco_feedback/camera/euler/roll", angles.roll);
	_nh.setParam("/aruco_feedback/camera/euler/pitch", angles.pitch);
	_nh.setParam("/aruco_feedback/camera/euler/yaw", angles.yaw);
	//return angles;
}

//------------------------------------------------------------------------------------------------------------------------------//

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "control_loop");
	moveit::planning_interface::MoveGroupInterface move_group("all");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface::Plan global_plan;
	
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("all");

	const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
	//const std::vector<std::string> predefined_poses = move_group.getNamedTargets();
	//cout<<"~~~~~~~~~~~~~~~~~~~~"<<predefined_poses[1]<<"\n";

	ros::NodeHandle _nh;
	ros::AsyncSpinner spinner(4); 
	control::CONTROL_SUBSCRIBER control_subscriber = control::CONTROL_SUBSCRIBER(_nh);
	//control::PLAN_RESULT_SUBSCRIBER plan_subscriber = control::PLAN_RESULT_SUBSCRIBER(_nh);
	control::EXECUTE_RESULT_SUBSCRIBER execute_subscriber = control::EXECUTE_RESULT_SUBSCRIBER(_nh);
//add calls here for execution subscriber & plan subscriber for use with try/catch statements for planning and executing
	bool remoteState;
	eulerAngles euler_angles;
	//_nh.getParam("/remoteControlBoom", remoteState);
	//printf("------------------------------------------------------------------------");
	//printf(remoteState ? "primed.\n" : "unprimed.\n");
	spinner.start();
	//const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("all");
//	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	double t0 = ros::Time::now().toSec();
	double t0_2 = ros::Time::now().toSec();
	_nh.setParam("/control/directionOfSurface", buildingDirection);
	while(ros::ok()){
		//plan_subscriber.subscribe();
		std::tuple <int, int> execution_sub_output = execute_subscriber.subscribe(); //working...make this useful. 
		executionResult = std::get<0>(execution_sub_output);
		newExecutionResult = std::get<1>(execution_sub_output);
		//add stateUpdater function here for more in depth state advancement checks.
		double elapsedTime;
		_nh.getParam("/control/elapsedTime", elapsedTime);
		if((state == 3 || state == 0) && ros::Time::now().toSec()-t0_2 >= elapsedTime){ //monitor how important/useful this is
			t0_2 = ros::Time::now().toSec();
			//Aruco-camera frame transform updating
			std::tuple <geometry_msgs::Pose, geometry_msgs::Pose> aruco_output = getArucoData(_nh, camera, aruco);
			camera = std::get<0>(aruco_output);
			aruco = std::get<1>(aruco_output);
			q2e(_nh, camera);
			//printing camera pose
/*
			printf("			-CAMERA-	time: %f.\n", ros::Time::now().toSec()-t0);
			printf("Position -> x: %f, y: %f, z: %f.\n", camera.position.x, camera.position.y, camera.position.z);
			printf("Orientation -> x: %f, y: %f, z: %f, w: %f.\n", camera.orientation.x, camera.orientation.y, camera.orientation.z, camera.orientation.w);
		//printf("Euler Angles -> roll: %f, pitch: %f, yaw %f. \n", euler_angles.roll, euler_angles.pitch, euler_angles.yaw);
*/
		}
		//--//

		EEFplotting(_nh, move_group, t0);
		_nh.setParam("/control/state", state);
		controller_output = control_subscriber.subscribe();
		Select = get<0>(controller_output);
		Start = get<1>(controller_output);
		PS = get<2>(controller_output);

		if(Select || Start || PS){
			//printf("Select: %d, start: %d\n", Select, Start);
			stateHandler(_nh, move_group, joint_model_group, global_plan);
		}
		if(enablePainting) performPainting(move_group);

		if(newExecutionResult){
			stateUpdater(_nh, execute_subscriber, executionResult);
		}
	}
	ros::waitForShutdown();
	return 0;
}

#include<hinowa_control/control_subscriber.h>
#include<hinowa_control/plan_result_subscriber.h>
#include<hinowa_control/execute_result_subscriber.h>
#include <fstream>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


using namespace std; 

std::tuple<int,int,int> controller_output;
int Select; int Start; int PS;
bool homePrimed = false; //Ready to go home
bool primed = false; //Ready to execute
int state = 0;
//States 
//0 initalise and base drive
//1 self level
//2 rotate to wall 
//3 camera codes 
//4 move up to wall top
//5 paint down
//6 retract 
//7 unlevel

//General Actions in each state
//-2 return to home execute
//-1 plans return to home
//0 plans to current position ie cleans the system
//1 plan to next state 
//2 execute next state

int SOT = 0; //state of trajectory
const string stateInfo[] = {"Remote track and stabiliser control", "Autonomous levelling", "Rotate to face wall", "Aruco code detection", "Move to top of building", "Apply paint to surface", "Retract from wall", "Autonomous unlevelling"};

//double buildingHeight; // Not in use
//double distanceFromWall; //Not in use 
//#define buildingDirection 1  //-1 for left : 1 for right
int buildingDirection = 0;
geometry_msgs::Point EEFxyz_prev;
double t_prev = 0;
geometry_msgs::Pose camera;
geometry_msgs::Pose aruco;
double tp0 = 0.0;
//double paintDelay = 0.0; //seconds //not in use
bool enablePainting = false; //Should the robot paint or just move
//moveit::planning_interface::MoveGroupInterface::Plan painting_plan;
int newExecutionResult;
int executionResult;
//-----------------------------------------------------------------------------------------------------------------------------------//
struct eulerAngles{
	double roll, pitch, yaw;
};


void print_state_info(int state_1, int state_2){
	cout<<"Current state: "<<stateInfo[state_1]<<". Next state: "<<stateInfo[state_2]<<", has been primed.\n";
}
void planToHome(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan){
	move_group.clearPathConstraints();
	move_group.setPlannerId("PTP");
	move_group.setNamedTarget("home");
	move_group.plan(global_plan);
}
void executeToHome(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, ros::NodeHandle _nh){
	_nh.setParam("/control/activeValveBlock", "top");
	bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("INFO","Execution command %s", result ? "succeeded.\n" : "FAILED.\n");
}
void planToCurrentPose(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int current_state){ //Plans to the current position to clean the system of error
	if(current_state == 7) print_state_info(7, 0);
	else print_state_info(current_state, current_state+1);

	move_group.setPlannerId("PTP");
	geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
	move_group.setPoseTarget(current_pose);
	move_group.plan(global_plan);
}

void setPlannerInfo(bool set_tolerence, bool move_to_retracted, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan){
	if(set_tolerence){
		move_group.setGoalTolerance(0.001); //If the error is larger than this value the code will shutdown and stop
	}
	move_group.setPlannerId("CIRC"); //Use the circular planner
	moveit_msgs::PositionConstraint interim_point;
	interim_point.link_name = "end"; //Select the end effector as the joint to position
	interim_point.weight = 1.0;
	interim_point.header.frame_id = "anchor";

	interim_point.constraint_region.primitive_poses.resize(1);
	
	interim_point.constraint_region.primitive_poses[0].position.x = 2.0*buildingDirection; //Distance from the base of the buling to the robot's centre
	interim_point.constraint_region.primitive_poses[0].position.y = 1.75; 
	interim_point.constraint_region.primitive_poses[0].position.z = 1.5; //Ground Height to start at
	
	moveit_msgs::Constraints interim_constraint;
	interim_constraint.name = "interim";
	interim_constraint.position_constraints.push_back(interim_point);

	move_group.setPathConstraints(interim_constraint);
	if(move_to_retracted){
		//Retracted pose
		target_pose.position.x = 3.00; 
		target_pose.position.y = 2.42043; 
		target_pose.position.z = 5.00; 
		target_pose.orientation.x = 0.0;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.z = 0.0;
		target_pose.orientation.w = 1;

		move_group.setPoseTarget(target_pose);
		move_group.plan(global_plan);
	}
}
void state0Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//track control, execute advances to autolvl
	if(action == -2){ //this state will have stabilisers in unpredictable positions - cannot allow return to home condition. 
	}	
	else if(action == -1){ //not necessary without the above being functional.
	}
	else if(action == 0){//not necessary without the above being functional.
		
		print_state_info(0, 1);
	}
	else if(action == 1){// _nh.setParam("/control/activeValveBlock", "bottom");
		print_state_info(0, 1);
	}
	else if(action == 2){
		cout<<"Executing state: "<<stateInfo[1]<<".\n";
		state = 1; //self level
		_nh.setParam("/control/activeValveBlock", "bottom");
		_nh.setParam("/control/levelled", 0);
		_nh.setParam("/level_sensor/firstSuccess", 1);
		_nh.setParam("/level_sensor/startingLevelling", 1);
	}
}
	
void state1Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){ //auto level, execute for rotate to wall
	if(action == -2){//this state will not be guaranteed to have the machine levelled - cannot allow return to home condition.
		cout<<"Executing state: "<<stateInfo[7]<<".\n";
		state = 7; //unlevel
		_nh.setParam("/control/activeValveBlock", "bottom");
		double distanceToWall;
	}
	else if(action == -1){//not necessary without the above being functional.
		print_state_info(1, 7);
	}
	else if(action == 0){//not necessary without the above being functional.
		planToCurrentPose(move_group, global_plan, 1);
	}
	else if(action == 1){
		print_state_info(1, 2);
//PATH SETUP
		//positive z is upward in world frame, positive y is to the left(?) in world frame from start view
		geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
		//move_group.setMaxAccelerationScalingFactor(0.2);
		//move_group.setMaxVelocityScalingFactor(0.3);
		move_group.setGoalTolerance(0.001);

//access joint_1 current position and perform conditionals to decide between CIRC and PTP planning.
		std::vector <double> joint_positions = move_group.getCurrentJointValues();
		//printf("Joint_1 position: %f.\n", joint_positions[0]);	

		if((-0.698132 < joint_positions[0] && joint_positions[0] < 0.698132) && (current_pose.position.z < 2.0)){ //CIRC plan case.
			setPlannerInfo(false, false, move_group, global_plan);
		}
		else move_group.setPlannerId("PTP");

//setup target pose
		geometry_msgs::Pose target_pose;
		target_pose.position.x = 3.00*   buildingDirection; //figure out these values using the lift with real CAN
		target_pose.position.y = 0.0; 
		target_pose.position.z = 5.00;    
		target_pose.orientation.x = 0.0;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.z = -0.7071*buildingDirection;
		target_pose.orientation.w = 0.7071;

		move_group.setPoseTarget(target_pose);
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
		bool virtualCan;
		_nh.getParam("/CAN/virtual", virtualCan);
		if(levelled && !virtualCan){ //Do Not comment these lines anymore, automatically works out if we are on virtual or real can
			cout<<"Executing state: "<<stateInfo[2]<<".\n";
			printf("State: %d.\n", state);
			//state = 2; //placeholder, needs to be replaced by stateUpdater function
			executeToHome(move_group, global_plan, _nh);
		}else if(!virtualCan){
			state = 1; //self level
		}
	}
	
}
void state2Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//face wall, execute to observe aruco codes
	if(action == -2){
		executeToHome(move_group, global_plan, _nh);
		//state = 0;
	}
	else if(action == -1){
		move_group.clearPathConstraints();
		geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
		//move_group.setMaxAccelerationScalingFactor(0.2);
		//move_group.setMaxVelocityScalingFactor(0.3);
		setPlannerInfo(true, true, move_group, global_plan);
	}
	else if(action == 0){
		planToCurrentPose(move_group, global_plan, 2);
	}
	else if(action == 1){
		print_state_info(2, 3);
	}
	else if(action == 2){
		cout<<"Executing state: "<<stateInfo[3]<<".\n";
		//State 3, check camera codes
		state = 3; //have a check for advancing this...possibly do elsewhere similar to movement states.
	}
}
void state3Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//Aruco code observe, execute for move to top
	if(action == -2){
		executeToHome(move_group, global_plan, _nh);
		//state = 0;
	}
	else if(action == -1){//go home
		move_group.clearPathConstraints();
		geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
		//move_group.setMaxAccelerationScalingFactor(1.0);
		//move_group.setMaxVelocityScalingFactor(0.6);
		//move_group.setMaxAccelerationScalingFactor(1.0);
		//move_group.setMaxVelocityScalingFactor(0.6);
		setPlannerInfo(true, true, move_group, global_plan);
	}
	else if(action == 0){
		planToCurrentPose(move_group, global_plan, 3);
	}
	else if(action == 1){
		print_state_info(3, 4);
		move_group.setPlannerId("LIN");
//SCALING FACTORS
		//move_group.setMaxAccelerationScalingFactor(0.5);
		//move_group.setMaxVelocityScalingFactor(0.7);
		move_group.setGoalTolerance(0.001);
//---//
//JOINT 1 CONSTRAINT 

//this isn't currently working - there is still small effort values being sent to joint_1. Try reworking this, or having a catch statement for valve actuation that says if the current state is = 5, do not allow actuation of joint_1.
/*
		std::vector <double> joint_positions = move_group.getCurrentJointValues();
		//printf("Joint_1 position: %f.\n", joint_positions[0]);

		moveit_msgs::JointConstraint lock_j1;
		lock_j1.joint_name = "joint_1";
		lock_j1.tolerance_above = 0.01;
		lock_j1.tolerance_below = 0.01;
		lock_j1.position = joint_positions[0];
//		if(buildingDirection == "left") lock_j1.position = 1.571; //figure out how to determine these values - should they read current
//		else if(buildingDirection == "right") lock_j1.position = -1.571; //j1 value and assign that...? investigate best solution.
		lock_j1.weight = 1.0;

		moveit_msgs::Constraints joint_constraint;
		joint_constraint.name = "Lock J1";
		joint_constraint.joint_constraints.push_back(lock_j1);

		move_group.setPathConstraints(joint_constraint);
*/
//---//
		//positive z is upward in world frame, positive y is to the left(?) in world frame from start view
		geometry_msgs::Pose pose = move_group.getCurrentPose().pose;


		pose.position.x = 3.00;                  //-3.25, eventually will be = camera sensed distance from aruco code
		pose.position.y = 0.0; 
		pose.position.z = 5.00; //4.9 //eventually will be = buildingHeight
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = -0.7071*buildingDirection; 
		pose.orientation.w = 0.7071;

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
		cout<<"Executing state: "<<stateInfo[4]<<".\n";
		printf("State: %d.\n", state);
		//state = 4; //replace by state updater
		_nh.setParam("/control/activeValveBlock", "top");
		move_group.asyncExecute(global_plan);
	}
}
void state4Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//top of building state, exec to paint
	if(action == -2){
		executeToHome(move_group, global_plan, _nh);
		//state = 0;
	}
	else if(action == -1){
		planToHome(move_group, global_plan);
	}
	else if(action == 0){
		planToCurrentPose(move_group, global_plan, 4);
	}
	else if(action == 1){
		print_state_info(4, 5;
//SCALING FACTORS
		move_group.setPlannerId("LIN");
		//move_group.setMaxAccelerationScalingFactor(0.5); //test these numbers
		//move_group.setMaxVelocityScalingFactor(0.5); //test these numbers
		//move_group.setMaxAccelerationScalingFactor(1.0); //test these numbers
		//move_group.setMaxVelocityScalingFactor(0.6); //test these numbers
		move_group.setGoalTolerance(0.001);
//---//
//JOINT 1 CONSTRAINT -----------------------> TRY REPLACING THE QUICK FIX IN HWI WITH A WORKING VERSION OF THIS FOR MOVING TO THE TOP, PAINTING, RETRACTING...

//this isn't currently working - there is still small effort values being sent to joint_1. Try reworking this, or having a catch statement for valve actuation that says if the current state is = 5, do not allow actuation of joint_1.
/*
		std::vector <double> joint_positions = move_group.getCurrentJointValues();
		//printf("Joint_1 position: %f.\n", joint_positions[0]);

		moveit_msgs::JointConstraint lock_j1;
		lock_j1.joint_name = "joint_1";
		lock_j1.tolerance_above = 0.01;
		lock_j1.tolerance_below = 0.01;
		lock_j1.position = joint_positions[0];
//		if(buildingDirection == "left") lock_j1.position = 1.571; //figure out how to determine these values - should they read current
//		else if(buildingDirection == "right") lock_j1.position = -1.571; //j1 value and assign that...? investigate best solution.
		lock_j1.weight = 1.0;

		moveit_msgs::Constraints joint_constraint;
		joint_constraint.name = "Lock J1";
		joint_constraint.joint_constraints.push_back(lock_j1);

		move_group.setPathConstraints(joint_constraint);
*/
//---//

//new planning code attempt
		geometry_msgs::Pose pose = move_group.getCurrentPose().pose;

		pose.position.x = 3.00;//-3.35; //1.1, set desired final z to ""m.
		pose.position.z = 5.00 ;/1.1, set desired final z to ""m.

		move_group.setPoseTarget(pose);
		
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
			outfile<<"PAINTING MOVEMENT - point_index: "<<i<<"\n";
			for (unsigned j=0; j<numJoints; j++){
				outfile<<trajectory.joint_trajectory.joint_names[j]<<","<<trajectory_points[i].positions[j]<<","<<trajectory_points[i].velocities[j]<<","<<trajectory_points[i].accelerations[j]<<",";
			}
			outfile<<"\n";
		}
		outfile.close();

	}
	else if(action == 2){
		cout<<"Executing state: "<<stateInfo[5]<<".\n";
		printf("STATE: %d. \n", state);
		//state = 5;//replace by state updater
		_nh.setParam("/control/activeValveBlock", "top");
		tp0 = ros::Time::now().toSec();
		enablePainting = true; 
		//painting_plan = global_plan;
		_nh.setParam("/control/ballValve", 1);
		printf("Valve open.\n");

		double distanceToWall;
		_nh.getParam("/control/distanceToWall", distanceToWall);
		printf("Distance from wall: %fcm.\n", distanceToWall);
/*
		//execute cartesian movement
		moveit::planning_interface::MoveItErrorCode execute_outcome;  	
		execute_outcome = move_group.asyncExecute(global_plan); //uncomment to execute
*/
	}
}

//Paint the wall 
void state5Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group,  moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//painting state, execute to retract

	if(action == -2){
		executeToHome(move_group, global_plan, _nh);
		//state = 0;
	}
	else if(action == -1){ 
		planToHome(move_group, global_plan);
	}
	else if(action == 0){ //Plan to current position to clear any accumilated errors
		planToCurrentPose(move_group, global_plan, 5);
	}

	else if(action == 1){ //Plan the next movement
		print_state_info(5, 6);
		move_group.setPlannerId("PTP"); //Point to point planner
		//move_group.setMaxAccelerationScalingFactor(0.2); //test these numbers
		//move_group.setMaxVelocityScalingFactor(0.5); //test these numbers
		move_group.setGoalTolerance(0.001); //If encoder error is larger than this, code will stop for saftey
		geometry_msgs::Pose retract_pose = move_group.getCurrentPose().pose;

		retract_pose.position.x = 3.00*buildingDirection;
		retract_pose.position.y = 0.0;
		retract_pose.position.z = 5.00;    

		move_group.setPoseTarget(retract_pose);
	}

	else if(action == 2){ //Execute the plan
		cout<<"Executing state: "<<stateInfo[6]<<".\n";
		bool result = (move_group.plan(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("INFO","Visualizing plan 1 (pose goal): %s", result ? "succeeded" : "FAILED");
		move_group.asyncExecute(global_plan);
	}
}


void state6Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//retracted state, execute to unlevel
	if(action == -2){
		executeToHome(move_group, global_plan, _nh);
		//state = 0;
	}
	else if(action == -1){
		move_group.clearPathConstraints();
		geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
		//move_group.setMaxAccelerationScalingFactor(0.2);
		//move_group.setMaxVelocityScalingFactor(0.3);
		setPlannerInfo(true, true, move_group, global_plan);
		
	}
	else if(action == 0){ //Plan to current position
		planToCurrentPose(move_group, global_plan, 6);
	}
	else if(action == 1){ //Plan to next state
		print_state_info(6, 7);
		//nothing performed here currently, should probably have checks to see if enabling destabilising is safe ie. joint positions.
	}
	else if(action == 2){ //Exceute next state
		cout<<"Executing state: "<<stateInfo[7]<<".\n";
		state = 7; //Unlevel the robot
		_nh.setParam("/control/activeValveBlock", "bottom");
		double distanceToWall;
		_nh.getParam("/control/distanceToWall", distanceToWall);
		printf("Distance from wall: %fcm.\n", distanceToWall);
	}
}

void performPainting(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan){
	double t_now = ros::Time::now().toSec();
	//if((t_now - tp0) > 0){ //uncomment this if statement and increase value from 0 if a delay is needed (sprayers turning on late).
		//execute cartesian movement
		moveit::planning_interface::MoveItErrorCode execute_outcome;  	
		execute_outcome = move_group.asyncExecute(global_plan); //uncomment to execute	
		enablePainting = false;
	//}
}


void state7Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//unlevel state, execute for return to 0
	if(action == -2){
/*
		_nh.setParam("/control/activeValveBlock", "top");
		bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		printf(result ? "succeeded\n" : "failed\n");
		state = 0;
*/
	}
	else if(action == -1){
/*
		move_group.setPlannerId("PTP");
		move_group.setNamedTarget("home");
		move_group.plan(global_plan);
*/
	}
	else if(action == 0){
		planToCurrentPose(move_group, global_plan, 7);
	}
	else if(action == 1){
		print_state_info(7, 0);
		//nothing performed here currently, should probably have checks to see if enabling destabilising is safe ie. joint positions.
	}
	else if(action == 2){
		cout<<"Executing state: "<<stateInfo[0]<<".\n";
		state = 0; //Drive the robot and initalise
		_nh.setParam("/control/activeValveBlock", "bottom");

	}
}

//-----------------------------------------------------------------------------------------------------------------------------------//

void stateUpdater(ros::NodeHandle _nh, control::EXECUTE_RESULT_SUBSCRIBER execute_subscriber, int executionResult, std::string actionType){
	cout<<"Action type: "<<actionType<<".\n";	
	printf("Execution result: %d.\n",executionResult);
	bool virtualCan;
	_nh.getParam("/CAN/virtual", virtualCan);

	if(virtualCan) state++;
	else{
		if(executionResult){
			if(actionType == "Executing advance"){
				state++;
			}
			else if(actionType == "Executing home"){
				state = 1; //Initalise the robot and switch to the bottom block
			}
		}

	}
	printf("State: %d.\n", state);
}


std::string stateHandler(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan)
{
		std::string actionType;
		int Send_Action;
		if(Select && !Start){//planning.
			homePrimed = false;
			primed = !primed;

			if(!primed){
				actionType = "Unpriming advance";				
			}else if(primed){
				actionType = "Priming advance";
			}
			Send_Action = primed;
			
//			cout<<"Current state: "<<stateInfo[state]<<". Next state: "<<stateInfo[nextState]<<", has been ";
//			printf(primed ? "primed.\n" : "unprimed.\n");
		}//end of planning conditional
		else if(PS && !Select && !Start){//planning for return to home condition for all states.
			primed = false; 
			homePrimed = !homePrimed;
			
			if(!homePrimed){
				actionType = "Unpriming home";
			}else if(homePrimed){
				actionType = "Priming home";
			}
			Send_Action = -1*homePrimed;
		}

		else if(!Select && Start){
			//state = nextState; //move this to new state advancement function. 
			//cout<<"Executing state: "<<stateInfo[state]<<".\n";
			if(primed && !homePrimed){
				Send_Action = 2;
				primed = false;
				actionType = "Executing advance";
			}else if(!primed && homePrimed){
				Send_Action = -2;
				homePrimed = false;
				actionType = "Executing home";
			}
		}
		switch(state){
			case 0 : state0Action(_nh, move_group, global_plan, Send_Action);
				 break;
			case 1 : state1Action(_nh, move_group, global_plan, Send_Action);
				 break;
			case 2 : state2Action(_nh, move_group, global_plan, Send_Action);
				 break;
			case 3 : state3Action(_nh, move_group, global_plan, Send_Action);
				 break;
			case 4 : state4Action(_nh, move_group, global_plan, Send_Action);
				 break;
			case 5 : state5Action(_nh, move_group, global_plan, Send_Action);
				 break;
			case 6 : state6Action(_nh, move_group, global_plan, Send_Action);
				 break;
			case 7 : state7Action(_nh, move_group, global_plan, Send_Action);
				 break;
		}
			
	return actionType;
};//end of stateHandler

//-----------------------------------------------------------------------------------------------------------------------------------//

double EEFplotting(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, double t0, std::string actionType){
		if(state == 4 && actionType == "Executing advance"){ //CHECK THIS STATE NUM AFTER CHANGES
			_nh.getParam("/control/SOT", SOT);
			geometry_msgs::Point EEFxyz = move_group.getCurrentPose().pose.position;
			if(!enablePainting){//perform painting function has been called, delay expended and enablePainting set to false. Allow spraying.	
				if((SOT == 1) && (EEFxyz.z > (1.5+1.0))){
					_nh.setParam("/control/ballValve", 1);
					//printf("Valve open.\n");
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


			}
			return t_prev;
			
		}
}
/*
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
*/
/*
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

//alt method
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
//---//
	angles.roll = angles.roll*180.0/M_PI;	
	angles.pitch = angles.pitch*180.0/M_PI;
	angles.yaw = angles.yaw*180.0/M_PI;
	
	_nh.setParam("/aruco_feedback/camera/euler/roll", angles.roll);
	_nh.setParam("/aruco_feedback/camera/euler/pitch", angles.pitch);
	_nh.setParam("/aruco_feedback/camera/euler/yaw", angles.yaw);
	//return angles;
}
*/
//------------------------------------------------------------------------------------------------------------------------------//

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "control_loop");
	moveit::planning_interface::MoveGroupInterface move_group("all");
	//poses = definePoses(move_group);
	move_group.setMaxAccelerationScalingFactor(1.0);
	move_group.setMaxVelocityScalingFactor(0.5);//0.6
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface::Plan global_plan;
	
//	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
//	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

//	kinematic_state->setToDefaultValues();
//	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("all");

//	const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

	ros::NodeHandle _nh;
	ros::AsyncSpinner spinner(4); 
	control::CONTROL_SUBSCRIBER control_subscriber = control::CONTROL_SUBSCRIBER(_nh);
	//control::PLAN_RESULT_SUBSCRIBER plan_subscriber = control::PLAN_RESULT_SUBSCRIBER(_nh);
	control::EXECUTE_RESULT_SUBSCRIBER execute_subscriber = control::EXECUTE_RESULT_SUBSCRIBER(_nh);
//add calls here for execution subscriber & plan subscriber for use with try/catch statements for planning and executing
	//bool remoteState;
	//eulerAngles euler_angles;
	//_nh.getParam("/remoteControlBoom", remoteState);
	//printf("------------------------------------------------------------------------");
	//printf(remoteState ? "primed.\n" : "unprimed.\n");
	spinner.start();
	//const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("all");
//	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	double t0 = ros::Time::now().toSec();
	double t0_2 = t0;
	std::string actionType;

//Get parameter for direction of building, check if it is a valid value, throw runtime error if it isn't.
	if(_nh.getParam("/control/directionOfSurface", buildingDirection) == "left") buildingDirection = -1;
	else if(_nh.getParam("/control/directionOfSurface", buildingDirection) == "right") buildingDirection = 1;
	if(buildingDirection == 0)
		throw std::runtime_error("Invalid Building Direction. Enter either left or right in /hinowa_control/control.yaml.\n");

	while(ros::ok()){
		//plan_subscriber.subscribe();
		std::tuple <int, int> execution_sub_output = execute_subscriber.subscribe();
		executionResult = std::get<0>(execution_sub_output);
		newExecutionResult = std::get<1>(execution_sub_output);
		double elapsedTime;
		_nh.getParam("/control/elapsedTime", elapsedTime);
/*		if((state == 3 || state == 0) && ros::Time::now().toSec()-t0_2 >= elapsedTime){ //monitor how important/useful this is
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

		}
*/
		//--//
		_nh.setParam("/control/state", state);
		controller_output = control_subscriber.subscribe();
		Select = get<0>(controller_output);
		Start = get<1>(controller_output);
		PS = get<2>(controller_output);

		if(Select || Start || PS){
			//printf("Select: %d, start: %d\n", Select, Start);
			actionType = stateHandler(_nh, move_group, global_plan);
		}
		if(enablePainting) performPainting(move_group, global_plan);
		EEFplotting(_nh, move_group, t0, actionType);

		if(newExecutionResult){
			stateUpdater(_nh, execute_subscriber, executionResult, actionType);
		}
	}
	ros::waitForShutdown();
	return 0;
}

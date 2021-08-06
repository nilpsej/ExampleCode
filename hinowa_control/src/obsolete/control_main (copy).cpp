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
bool homePrimed = false;
bool primed = false;
int state = 0;
int SOT = 0; //state of trajectory
const string stateInfo[] = {"Remote track and stabiliser control", "Autonomous levelling", "Rotate to face surface", "Aruco code detection", "Move to top of surface", "Apply paint to surface", "Retract from surface", "Autonomous unlevelling"};

double surfaceHeight;
double distanceFromsurface;
const string surfaceDirection = "left";
geometry_msgs::Point EEFxyz_prev;
double t_prev = 0;
double tp0 = 0.0;
double paintDelay = 0.0;
bool enablePainting = false;
//moveit::planning_interface::MoveGroupInterface::Plan painting_plan;
int newExecutionResult;
int executionResult;
bool virtualCAN;
//-----------------------------------------------------------------------------------------------------------------------------------//
struct eulerAngles
{
    double roll, pitch, yaw;
};

//constant velocity function
void setAvgCartesianSpeed(moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string end_effector, const double speed)
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
 
    int num_waypoints = plan.trajectory_.joint_trajectory.points.size();                                //gets the number of waypoints in the trajectory
    const std::vector<std::string> joint_names = plan.trajectory_.joint_trajectory.joint_names;    //gets the names of the joints being updated in the trajectory
 
    //set joint positions of zeroth waypoint
    kinematic_state->setVariablePositions(joint_names, plan.trajectory_.joint_trajectory.points.at(0).positions);
 
    Eigen::Affine3d current_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
    Eigen::Affine3d next_end_effector_state;
    double euclidean_distance, new_timestamp, old_timestamp, q1, q2, q3, dt1, dt2, v1, v2, a;
    trajectory_msgs::JointTrajectoryPoint *prev_waypoint, *curr_waypoint, *next_waypoint;
 
    for(int i = 0; i < num_waypoints - 1; i++)      //loop through all waypoints
    {
        curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);
        next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i+1);
         
        //set joints for next waypoint
        kinematic_state->setVariablePositions(joint_names, next_waypoint->positions);
 
        //do forward kinematics to get cartesian positions of end effector for next waypoint
        next_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
 
        //get euclidean distance between the two waypoints
        euclidean_distance = pow(pow(next_end_effector_state.translation()[0] - current_end_effector_state.translation()[0], 2) + 
                            pow(next_end_effector_state.translation()[1] - current_end_effector_state.translation()[1], 2) + 
                            pow(next_end_effector_state.translation()[2] - current_end_effector_state.translation()[2], 2), 0.5);
 
        new_timestamp = curr_waypoint->time_from_start.toSec() + (euclidean_distance / speed);      //start by printing out all 3 of these!
        old_timestamp = next_waypoint->time_from_start.toSec();

        //update next waypoint timestamp & joint velocities/accelerations if joint velocity/acceleration constraints allow
        if(new_timestamp > old_timestamp)
            next_waypoint->time_from_start.fromSec(new_timestamp);
        else
        {
		//printf("New timestamp: %f, Old timestamp: %f, euclidean distance: %f, speed: %f.\n", new_timestamp, old_timestamp, euclidean_distance, speed);
		next_waypoint->time_from_start.fromSec(new_timestamp);
            //ROS_WARN_NAMED("setAvgCartesianSpeed", "Average speed is too fast. Moving as fast as joint constraints allow.");
        }
         
        //update current_end_effector_state for next iteration
        current_end_effector_state = next_end_effector_state;
    }
     
    //now that timestamps are updated, update joint velocities/accelerations (used updateTrajectory from iterative_time_parameterization as a reference)
    for(int i = 0; i < num_waypoints; i++)
    {
        curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);            //set current, previous & next waypoints
        if(i > 0)
            prev_waypoint = &plan.trajectory_.joint_trajectory.points.at(i-1);
        if(i < num_waypoints-1)
            next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i+1);
 
        if(i == 0)          //update dt's based on waypoint (do this outside of loop to save time)
            dt1 = dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
        else if(i < num_waypoints-1)
        {
            dt1 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();
            dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
        }
        else
            dt1 = dt2 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();
 
        for(int j = 0; j < joint_names.size(); j++)     //loop through all joints in waypoint
        {
            if(i == 0)                      //first point
            {
                q1 = next_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = q1;
            }
            else if(i < num_waypoints-1)    //middle points
            {
                q1 = prev_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = next_waypoint->positions.at(j);
            }
            else                            //last point
            {
                q1 = prev_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = q1;
            }
 
            if(dt1 == 0.0 || dt2 == 0.0)
                v1 = v2 = a = 0.0;
            else
            {
                v1 = (q2 - q1)/dt1;
                v2 = (q3 - q2)/dt2;
                a = 2.0*(v2 - v1)/(dt1+dt2);
            }
 
            //actually set the velocity and acceleration
            curr_waypoint->velocities.at(j) = (v1+v2)/2;
            curr_waypoint->accelerations.at(j) = a;
        }
    }
	//return plan;
}

//--------------------------------------------------------------------------------------------------------------------------------------------//

void state0Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//track control, execute advances to autolvl
	if(action == -2){ //this state will have stabilisers in unpredictable positions - cannot allow return to home condition. 
	}	
	if(action == -1){ //not necessary without the above being functional.
	}
	else if(action == 0){//not necessary without the above being functional.
		cout<<"Current state: "<<stateInfo[0]<<". Next state: "<<stateInfo[1]<<", has been unprimed.\n";
	}
	else if(action == 1){// _nh.setParam("/control/activeValveBlock", "bottom");
		cout<<"Current state: "<<stateInfo[0]<<". Next state: "<<stateInfo[1]<<", has been primed.\n";
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
	
void state1Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){ //auto level, execute for rotate to surface
	if(action == -2){//this state will not be guaranteed to have the machine levelled - cannot allow return to home condition.
		cout<<"Executing state: "<<stateInfo[7]<<".\n";
		state = 7; 
		_nh.setParam("/control/activeValveBlock", "bottom");
		double distanceToSurface;
	}
	else if(action == -1){//not necessary without the above being functional.
		cout<<"Current state: "<<stateInfo[1]<<". Next state: "<<stateInfo[7]<<", has been primed.\n";
	}	
	else if(action == 0){//not necessary without the above being functional.
		cout<<"Current state: "<<stateInfo[1]<<". Next state: "<<stateInfo[2]<<", has been unprimed.\n";

		move_group.setPlannerId("PTP");
		geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
		move_group.setPoseTarget(current_pose);
		move_group.plan(global_plan);
	}
	else if(action == 1){
		cout<<"Current state: "<<stateInfo[1]<<". Next state: "<<stateInfo[2]<<", has been primed.\n"; //print out of whats happening.
//PATH SETUP
		//positive z is upward in world frame, positive y is to the left(?) in world frame from start view
		geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
		//move_group.setMaxAccelerationScalingFactor(0.2);
		//move_group.setMaxVelocityScalingFactor(0.3);
		move_group.setGoalTolerance(0.001);

//access joint_1 current position and perform conditionals to decide between CIRC and PTP planning.
		std::vector <double> joint_positions = move_group.getCurrentJointValues();

		move_group.clearPathConstraints();
		//move_group.setPlannerId("PTP");
		move_group.setMaxAccelerationScalingFactor(0.5);
		move_group.setMaxVelocityScalingFactor(0.5);
		if(surfaceDirection == "left") move_group.setNamedTarget("face_wall_left"); 
		else if(surfaceDirection == "right") move_group.setNamedTarget("face_wall_right");
/*
		std::vector<double> joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		if(surfaceDirection == "left") joint_values[0] = 1.57071;
		else if(surfaceDirection == "right") joint_values[0] = -1.57071;
		move_group.setJointValueTarget(joint_values);
*/
		move_group.plan(global_plan);
		//printf("Joint_1 position: %f.\n", joint_positions[0]);	
/*
		if((-0.698132 < joint_positions[0] && joint_positions[0] < 0.698132) && (current_pose.position.z < 2.0)){ //CIRC plan case.
			move_group.setPlannerId("CIRC");
			moveit_msgs::PositionConstraint interim_point;
			interim_point.link_name = "end";
			interim_point.weight = 1.0;
			interim_point.header.frame_id = "anchor";
		
	//		double interim_z = fabs(current_pose.position.z+target_pose.position.z)/2.0;
	//		printf("Interim z: %f.\n", interim_z);
			interim_point.constraint_region.primitive_poses.resize(1);
			if(surfaceDirection == "left") interim_point.constraint_region.primitive_poses[0].position.x = -2.0;
			else if(surfaceDirection == "right") interim_point.constraint_region.primitive_poses[0].position.x = 2.0;
			interim_point.constraint_region.primitive_poses[0].position.y = 1.75;
			interim_point.constraint_region.primitive_poses[0].position.z = 1.5;

			moveit_msgs::Constraints interim_constraint;
			interim_constraint.name = "interim";
			interim_constraint.position_constraints.push_back(interim_point);

			move_group.setPathConstraints(interim_constraint);
		}

		else move_group.setPlannerId("PTP");

//setup target pose
		geometry_msgs::Pose target_pose;
		target_pose.position.y = 0.0;
		target_pose.position.z = 0.912296;
		target_pose.orientation.x = 0.0;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.w = 0.7071;

		if(surfaceDirection == "left"){
			target_pose.position.x = -2.42043;
			target_pose.orientation.z = 0.7071;
		}
		else if(surfaceDirection == "right"){
			target_pose.position.x = 2.42043;
			target_pose.orientation.z = -0.7071;
		}

		move_group.setPoseTarget(target_pose);
		bool result = (move_group.plan(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("INFO","Visualizing plan 1 (pose goal) %s", result ? "succeeded" : "FAILED");
*/

//printing out trajectory plan
		moveit_msgs::RobotTrajectory trajectory = global_plan.trajectory_;
		std::ofstream outfile("/home/josh/workspace/src/hinowacpp/points.csv",std::ios::app);
         	//std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajectory_points1;  
		std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = trajectory.joint_trajectory.points; 

         	std::vector<int>::size_type size = trajectory_points.size(); 
		std::vector<int>::size_type numJoints = trajectory.joint_trajectory.joint_names.size();
		
		for(unsigned i = 0; i<size; i++){
			outfile<<"ROTATE TO FACE surface - point_index: "<<i<<"\n";
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
		if(virtualCAN){
				cout<<"Executing state: "<<stateInfo[2]<<".\n";
				printf("State: %d.\n", state);
				_nh.setParam("/control/activeValveBlock", "top");
				bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
				ROS_INFO_NAMED("INFO","Execution command %s", result ? "succeeded.\n" : "FAILED.\n");
		}
		else{
			if(levelled){ //UNCOMMENT THESE LINES FOR SAFETY WHEN OPERATING, COMMENT FOR VIRTUAL CAN TESTS
				cout<<"Executing state: "<<stateInfo[2]<<".\n";
				printf("State: %d.\n", state);
				//state = 2; //placeholder, needs to be replaced by stateUpdater function
				_nh.setParam("/control/activeValveBlock", "top");
				bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
				ROS_INFO_NAMED("INFO","Execution command %s", result ? "succeeded.\n" : "FAILED.\n");
			}
			else state = 1;
		}
	}
	
}
void state2Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//face surface, execute to observe aruco codes
	if(action == -2){
		_nh.setParam("/control/activeValveBlock", "top");
		bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("INFO","Execution command %s", result ? "succeeded.\n" : "FAILED.\n");
		//state = 0;
	}
	else if(action == -1){
		move_group.clearPathConstraints();
		geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
		//move_group.setMaxAccelerationScalingFactor(0.2);
		//move_group.setMaxVelocityScalingFactor(0.3);
		move_group.setGoalTolerance(0.001);
		move_group.setPlannerId("CIRC");
		moveit_msgs::PositionConstraint interim_point;
		interim_point.link_name = "end";
		interim_point.weight = 1.0;
		interim_point.header.frame_id = "anchor";

		interim_point.constraint_region.primitive_poses.resize(1);
		if(surfaceDirection == "left") interim_point.constraint_region.primitive_poses[0].position.x = -2.0;
		else if(surfaceDirection == "right") interim_point.constraint_region.primitive_poses[0].position.x = 2.0;
		interim_point.constraint_region.primitive_poses[0].position.y = 1.75;
		interim_point.constraint_region.primitive_poses[0].position.z = 1.5;

		moveit_msgs::Constraints interim_constraint;
		interim_constraint.name = "interim";
		interim_constraint.position_constraints.push_back(interim_point);

		move_group.setPathConstraints(interim_constraint);
		
		//home pose
		target_pose.position.x = 0.0;
		target_pose.position.y = 2.43; 
		target_pose.position.z = 0.935;
		target_pose.orientation.x = 0.0;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.z = 0.0;
		target_pose.orientation.w = 1;

		move_group.setPoseTarget(target_pose);
		move_group.plan(global_plan);
	}
	else if(action == 0){
		cout<<"Current state: "<<stateInfo[2]<<". Next state: "<<stateInfo[3]<<", has been unprimed.\n";

		move_group.setPlannerId("PTP");
		geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
		move_group.setPoseTarget(current_pose);
		move_group.plan(global_plan);
	}
	else if(action == 1){
		cout<<"Current state: "<<stateInfo[2]<<". Next state: "<<stateInfo[3]<<", has been primed.\n";
	}
	else if(action == 2){
		cout<<"Executing state: "<<stateInfo[3]<<".\n";
		state = 3; //have a check for advancing this...possibly do elsewhere similar to movement states.
	}
}
void state3Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//Aruco code observe, execute for move to top
	if(action == -2){
		_nh.setParam("/control/activeValveBlock", "top");
		bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("INFO","Execution command %s", result ? "succeeded.\n" : "FAILED.\n");
		//state = 0;
	}
	else if(action == -1){//go home
		move_group.clearPathConstraints();
		geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
		//move_group.setMaxAccelerationScalingFactor(1.0);
		//move_group.setMaxVelocityScalingFactor(0.6);
		//move_group.setMaxAccelerationScalingFactor(1.0);
		//move_group.setMaxVelocityScalingFactor(0.6);
		move_group.setGoalTolerance(0.001);
		move_group.setPlannerId("CIRC");
		moveit_msgs::PositionConstraint interim_point;
		interim_point.link_name = "end";
		interim_point.weight = 1.0;
		interim_point.header.frame_id = "anchor";

		interim_point.constraint_region.primitive_poses.resize(1);
		if(surfaceDirection == "left") interim_point.constraint_region.primitive_poses[0].position.x = -2.0;
		else if(surfaceDirection == "right") interim_point.constraint_region.primitive_poses[0].position.x = 2.0;
		interim_point.constraint_region.primitive_poses[0].position.y = 1.75;
		interim_point.constraint_region.primitive_poses[0].position.z = 1.5;

		moveit_msgs::Constraints interim_constraint;
		interim_constraint.name = "interim";
		interim_constraint.position_constraints.push_back(interim_point);

		move_group.setPathConstraints(interim_constraint);
		
		//home pose
		target_pose.position.x = 0.0;
		target_pose.position.y = 2.43; 
		target_pose.position.z = 0.935;
		target_pose.orientation.x = 0.0;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.z = 0.0;
		target_pose.orientation.w = 1;

		move_group.setPoseTarget(target_pose);
		move_group.plan(global_plan);
	}
	else if(action == 0){
		cout<<"Current state: "<<stateInfo[3]<<". Next state: "<<stateInfo[4]<<", has been unprimed.\n";

		move_group.setPlannerId("PTP");
		geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
		move_group.setPoseTarget(current_pose);
		move_group.plan(global_plan);
	}
	else if(action == 1){
		cout<<"Current state: "<<stateInfo[3]<<". Next state: "<<stateInfo[4]<<", has been primed.\n";
		//move_group.setPlannerId("LIN");
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
//		if(surfaceDirection == "left") lock_j1.position = 1.571; //figure out how to determine these values - should they read current
//		else if(surfaceDirection == "right") lock_j1.position = -1.571; //j1 value and assign that...? investigate best solution.
		lock_j1.weight = 1.0;

		moveit_msgs::Constraints joint_constraint;
		joint_constraint.name = "Lock J1";
		joint_constraint.joint_constraints.push_back(lock_j1);

		move_group.setPathConstraints(joint_constraint);
*/
//---//
		//positive z is upward in world frame, positive y is to the left(?) in world frame from start view
		geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
		std::vector<geometry_msgs::Pose> waypoints;
		//waypoints.push_back(target_pose); //add to waypoint list
		target_pose.position.z = 6.0;
		target_pose.position.x = -3.375;
/*
		double distanceToSurface = 4.0;
		double surfaceHeight = 8.0;

		target_pose.position.y = 0.0; 
		target_pose.position.z = surfaceHeight; //eventually will be from camera/aruco code
		target_pose.orientation.x = 0.0;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.w = 0.7071;

		if(surfaceDirection == "left"){
			target_pose.position.x = -1*distanceToSurface;
			target_pose.orientation.z = 0.7071;
		}
		else if(surfaceDirection == "right"){
			target_pose.position.x = distanceToSurface;
			target_pose.orientation.z = -0.7071;
		}
*/
/*
		move_group.setPoseTarget(target_pose);
		
		bool result = (move_group.plan(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		
		if(virtualCAN) ROS_INFO_NAMED("INFO","Visualizing plan 1 (pose goal): %s", result ? "succeeded" : "FAILED");
		else{
			if(result) ROS_INFO_NAMED("INFO","Visualizing plan 1 (pose goal): %s", result ? "succeeded" : "FAILED");
			else throw std::runtime_error("Cartesian movement to top of surface cannot be planned. Exiting.\n");
		}
*/


		waypoints.push_back(target_pose);

		moveit_msgs::RobotTrajectory trajectory;
		
		const double jump_threshold = 0; //was 1000
 		const double eef_step = 0.01;
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

		setAvgCartesianSpeed(global_plan, "end", 0.2);//0.3


//printing out trajectory plan
//		moveit_msgs::RobotTrajectory trajectory = global_plan.trajectory_;
		std::ofstream outfile("/home/josh/workspace/src/hinowacpp/points.csv",std::ios::app);
         	//std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajectory_points1;  
		std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = trajectory.joint_trajectory.points; 

         	std::vector<int>::size_type size = trajectory_points.size(); 
		std::vector<int>::size_type numJoints = trajectory.joint_trajectory.joint_names.size();
		
		for(unsigned i = 0; i<size; i++){
			outfile<<"MOVE TO TOP OF surface - point_index: "<<i<<"\n";
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
void state4Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//top of surface state, exec to paint
	if(action == -2){
		_nh.setParam("/control/activeValveBlock", "top");
		bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("INFO","Execution command %s", result ? "succeeded.\n" : "FAILED.\n");
		//state = 0;
	}
	else if(action == -1){
		move_group.clearPathConstraints();
		move_group.setPlannerId("PTP");
		move_group.setNamedTarget("home");
		move_group.plan(global_plan);
	}
	else if(action == 0){
		cout<<"Current state: "<<stateInfo[4]<<". Next state: "<<stateInfo[5]<<", has been unprimed.\n";

		move_group.setPlannerId("PTP");
		geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
		move_group.setPoseTarget(current_pose);
		move_group.plan(global_plan);
	}
	else if(action == 1){
		cout<<"Current state: "<<stateInfo[4]<<". Next state: "<<stateInfo[5]<<", has been primed.\n";
//SCALING FACTORS
		//move_group.setPlannerId("LIN");
		//move_group.setMaxAccelerationScalingFactor(0.5); //test these numbers
		//move_group.setMaxVelocityScalingFactor(0.5); //test these numbers
		//move_group.setMaxAccelerationScalingFactor(0.7); //test these numbers
		//move_group.setMaxVelocityScalingFactor(0.4); //test these numbers
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
//		if(surfaceDirection == "left") lock_j1.position = 1.571; //figure out how to determine these values - should they read current
//		else if(surfaceDirection == "right") lock_j1.position = -1.571; //j1 value and assign that...? investigate best solution.
		lock_j1.weight = 1.0;

		moveit_msgs::Constraints joint_constraint;
		joint_constraint.name = "Lock J1";
		joint_constraint.joint_constraints.push_back(lock_j1);

		move_group.setPathConstraints(joint_constraint);
*/
//---//

//new planning code attempt
		geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
		std::vector<geometry_msgs::Pose> waypoints;
		target_pose.position.x = -3.55;
		target_pose.position.z = 2.0;

		waypoints.push_back(target_pose);
/*
		geometry_msgs::Pose retract_pose = move_group.getCurrentPose().pose;

		//if(surfaceDirection == "left") retract_pose.position.x = -2.42043;
		//else if(surfaceDirection == "right") retract_pose.position.x = 2.42043;
		retract_pose.position.x = -2.42043;
		retract_pose.position.y = 0.0;
		retract_pose.position.z = 0.912296;
		waypoints.push_back(retract_pose);
*/
		moveit_msgs::RobotTrajectory trajectory;
		
		const double jump_threshold = 0; //was 1000
 		const double eef_step = 0.01;
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

		setAvgCartesianSpeed(global_plan, "end", 0.45); //0.6
//		if(surfaceDirection == "left") pose.position.x = -3.0;
//		else if(surfaceDirection == "right") pose.position.x = 3.0;
/*
		move_group.setPoseTarget(target_pose);

		bool result = (move_group.plan(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		if(virtualCAN) ROS_INFO_NAMED("INFO","Visualizing plan 1 (pose goal): %s", result ? "succeeded" : "FAILED");
		else{
			if(result){
				//setAvgCartesianSpeed(global_plan, "end", 0.07);
				result = (move_group.plan(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
				ROS_INFO_NAMED("INFO","Visualizing plan 1 (pose goal): %s", result ? "succeeded" : "FAILED");
			}
			else throw std::runtime_error("Cartesian movement for painting surface cannot be planned. Exiting.\n");
		}
*/
		

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

		double distanceToSurface;
		_nh.getParam("/surface/distance", distanceToSurface);
		printf("Distance from surface: %fcm.\n", distanceToSurface);
/*
		//execute cartesian movement
		moveit::planning_interface::MoveItErrorCode execute_outcome;  	
		execute_outcome = move_group.asyncExecute(global_plan); //uncomment to execute
*/
	}
}

void state5Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group,  moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//painting state, execute to retract

	if(action == -2){
		_nh.setParam("/control/activeValveBlock", "top");
		bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("INFO","Execution command %s", result ? "succeeded.\n" : "FAILED.\n");
		//state = 0;
	}
	else if(action == -1){
		move_group.clearPathConstraints();
		move_group.setPlannerId("PTP");//may have to create PTP or CIRC conditionals here
		move_group.setNamedTarget("home");
		move_group.plan(global_plan);
	}
	else if(action == 0){
		cout<<"Current state: "<<stateInfo[5]<<". Next state: "<<stateInfo[6]<<", has been unprimed.\n";

		move_group.setPlannerId("PTP");
		geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
		move_group.setPoseTarget(current_pose);
		move_group.plan(global_plan);
	}

	else if(action == 1){
		cout<<"Current state: "<<stateInfo[5]<<". Next state: "<<stateInfo[6]<<", has been primed.\n";
		move_group.setPlannerId("PTP");
		//move_group.setMaxAccelerationScalingFactor(0.2); //test these numbers
		//move_group.setMaxVelocityScalingFactor(0.5); //test these numbers
		move_group.setGoalTolerance(0.001);
		geometry_msgs::Pose retract_pose = move_group.getCurrentPose().pose;

		if(surfaceDirection == "left") retract_pose.position.x = -2.42043;
		else if(surfaceDirection == "right") retract_pose.position.x = 2.42043;
		retract_pose.position.y = 0.0;
		retract_pose.position.z = 0.912296;

		move_group.setPoseTarget(retract_pose);
	}

	else if(action == 2){
		cout<<"Executing state: "<<stateInfo[6]<<".\n";
		bool result = (move_group.plan(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("INFO","Visualizing plan 1 (pose goal): %s", result ? "succeeded" : "FAILED");
		move_group.asyncExecute(global_plan);
	}
}


void state6Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan, int action){//retracted state, execute to unlevel
	if(action == -2){
		_nh.setParam("/control/activeValveBlock", "top");
		bool result = (move_group.asyncExecute(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("INFO","Execution command %s", result ? "succeeded.\n" : "FAILED.\n");
		//state = 0;
	}
	else if(action == -1){
		move_group.clearPathConstraints();
		geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
		//move_group.setMaxAccelerationScalingFactor(0.2);
		//move_group.setMaxVelocityScalingFactor(0.3);
		move_group.setGoalTolerance(0.001);
		move_group.setPlannerId("CIRC");
		moveit_msgs::PositionConstraint interim_point;
		interim_point.link_name = "end";
		interim_point.weight = 1.0;
		interim_point.header.frame_id = "anchor";

		interim_point.constraint_region.primitive_poses.resize(1);
		if(surfaceDirection == "left") interim_point.constraint_region.primitive_poses[0].position.x = -2.0;
		else if(surfaceDirection == "right") interim_point.constraint_region.primitive_poses[0].position.x = 2.0;
		interim_point.constraint_region.primitive_poses[0].position.y = 1.75;
		interim_point.constraint_region.primitive_poses[0].position.z = 1.5;

		moveit_msgs::Constraints interim_constraint;
		interim_constraint.name = "interim";
		interim_constraint.position_constraints.push_back(interim_point);

		move_group.setPathConstraints(interim_constraint);
		

		target_pose.position.x = 0.0; //figure out these values using the lift with real CAN
		target_pose.position.y = 2.43; 
		target_pose.position.z = 0.935;
		target_pose.orientation.x = 0.0;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.z = 0.0;
		target_pose.orientation.w = 1;

		move_group.setPoseTarget(target_pose);
		move_group.plan(global_plan);
	}
	else if(action == 0){
		cout<<"Current state: "<<stateInfo[6]<<". Next state: "<<stateInfo[7]<<", has been unprimed.\n";


		move_group.setPlannerId("PTP");
		geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
		move_group.setPoseTarget(current_pose);
		move_group.plan(global_plan);
	}
	else if(action == 1){
		cout<<"Current state: "<<stateInfo[6]<<". Next state: "<<stateInfo[7]<<", has been primed.\n";
		//nothing performed here currently, should probably have checks to see if enabling destabilising is safe ie. joint positions.
	}
	else if(action == 2){
		cout<<"Executing state: "<<stateInfo[7]<<".\n";
		state = 7; 
		_nh.setParam("/control/activeValveBlock", "bottom");
		double distanceToSurface;
		_nh.getParam("/control/distanceToSurface", distanceToSurface);
		printf("Distance from surface: %fcm.\n", distanceToSurface);
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
		cout<<"Current state: "<<stateInfo[7]<<". Next state: "<<stateInfo[0]<<", has been unprimed.\n";

		move_group.setPlannerId("PTP");
		geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
		move_group.setPoseTarget(current_pose);
		move_group.plan(global_plan);
	}
	else if(action == 1){
		cout<<"Current state: "<<stateInfo[7]<<". Next state: "<<stateInfo[0]<<", has been primed.\n";
		//nothing performed here currently, should probably have checks to see if enabling destabilising is safe ie. joint positions.
	}
	else if(action == 2){
		cout<<"Executing state: "<<stateInfo[0]<<".\n";
		state = 0;
		_nh.setParam("/control/activeValveBlock", "bottom");
		//Apply actions for raising stabilisers back off the ground - monitor the gyro while doing so to inform effort values. Once all lights are off, have a time measured application of full effort to raise them a reasonable amount to allow driving without collision.
	}
}

//-----------------------------------------------------------------------------------------------------------------------------------//

void stateUpdater(ros::NodeHandle _nh, control::EXECUTE_RESULT_SUBSCRIBER execute_subscriber, int executionResult, std::string actionType){
	cout<<"Action type: "<<actionType<<".\n";	
	printf("Execution result: %d.\n",executionResult);

	if(virtualCAN) state++;
	else{
//commented for display of operation

		if(executionResult){
			if(actionType == "Executing advance"){
				state++;
			}
			else if(actionType == "Executing home"){
				state = 1;
			}
		}

	}
	printf("State: %d.\n", state);
}


std::string stateHandler(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& global_plan)
{
		std::string actionType;

		if(Select && !Start){//planning.
			homePrimed = false;
			if(!primed) primed = true;
			else primed = false;
		
			if(!primed){
				actionType = "Unpriming advance";				
				switch(state){
				case 0 : state0Action(_nh, move_group, global_plan, 0);
					 break;
				case 1 : state1Action(_nh, move_group, global_plan, 0);
					 break;
				case 2 : state2Action(_nh, move_group, global_plan, 0);
					 break;
				case 3 : state3Action(_nh, move_group, global_plan, 0);
					 break;
				case 4 : state4Action(_nh, move_group, global_plan, 0);
					 break;
				case 5 : state5Action(_nh, move_group, global_plan, 0);
					 break;
				case 6 : state6Action(_nh, move_group, global_plan, 0);
					 break;
				case 7 : state7Action(_nh, move_group, global_plan, 0);
					 break;
				}//end of !primed switch.
			}//end of !primed if.
			else if(primed){
				actionType = "Priming advance";
				switch(state){
				case 0 : state0Action(_nh, move_group, global_plan, 1);
					 break;
				case 1 : state1Action(_nh, move_group, global_plan, 1);
					 break;
				case 2 : state2Action(_nh, move_group, global_plan, 1);
					 break;
				case 3 : state3Action(_nh, move_group, global_plan, 1);
					 break;
				case 4 : state4Action(_nh, move_group, global_plan, 1);
					 break;
				case 5 : state5Action(_nh, move_group, global_plan, 1);
					 break;
				case 6 : state6Action(_nh, move_group, global_plan, 1);
					 break;
				case 7 : state7Action(_nh, move_group, global_plan, 1);
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
				actionType = "Unpriming home";
				switch(state){
				case 0 : state0Action(_nh, move_group, global_plan, 0);
					 break;
				case 1 : state1Action(_nh, move_group, global_plan, 0);
					 break;
				case 2 : state2Action(_nh, move_group, global_plan, 0);
					 break;
				case 3 : state3Action(_nh, move_group, global_plan, 0);
					 break;
				case 4 : state4Action(_nh, move_group, global_plan, 0);
					 break;
				case 5 : state5Action(_nh, move_group, global_plan, 0);
					 break;
				case 6 : state6Action(_nh, move_group, global_plan, 0);
					 break;
				case 7 : state7Action(_nh, move_group, global_plan, 0);
					 break;
				}//end of !homePrimed switch.
			}
			else if(homePrimed){
				actionType = "Priming home";
				switch(state){
				case 0 : state0Action(_nh, move_group, global_plan, -1);
					 break;
				case 1 : state1Action(_nh, move_group, global_plan, -1);
					 break;
				case 2 : state2Action(_nh, move_group, global_plan, -1);
					 break;
				case 3 : state3Action(_nh, move_group, global_plan, -1);
					 break;
				case 4 : state4Action(_nh, move_group, global_plan, -1);
					 break;
				case 5 : state5Action(_nh, move_group, global_plan, -1);
					 break;
				case 6 : state6Action(_nh, move_group, global_plan, -1);
					 break;
				case 7 : state7Action(_nh, move_group, global_plan, -1);
					 break;
				}//end of homePrimed switch.
			}
		}

		else if(!Select && Start && primed && !homePrimed){//executing state advancement.
			primed = false;
			//state = nextState; //move this to new state advancement function. 
			//cout<<"Executing state: "<<stateInfo[state]<<".\n";
			actionType = "Executing advance";
			switch(state){
			case 0 : state0Action(_nh, move_group, global_plan, 2);
				 break;
			case 1 : state1Action(_nh, move_group, global_plan, 2);
				 break;
			case 2 : state2Action(_nh, move_group, global_plan, 2);
				 break;
			case 3 : state3Action(_nh, move_group, global_plan, 2);
				 break;
			case 4 : state4Action(_nh, move_group, global_plan, 2);
				 break;
			case 5 : state5Action(_nh, move_group, global_plan, 2);
				 break;
			case 6 : state6Action(_nh, move_group, global_plan, 2);
				 break;
			case 7 : state7Action(_nh, move_group, global_plan, 2);
				 break;
			}//end of executing if.
		}//end of execution conditional	

		else if(!Select && Start && !primed && homePrimed){//executing return to home.
			homePrimed = false;
			//state = nextState; //move this to new state advancement function.
//			cout<<"Executing state: "<<stateInfo[state]<<".\n";
			actionType = "Executing home";
			switch(state){
			case 0 : state0Action(_nh, move_group, global_plan, -2);
				 break;
			case 1 : state1Action(_nh, move_group, global_plan, -2);
				 break;
			case 2 : state2Action(_nh, move_group, global_plan, -2);
				 break;
			case 3 : state3Action(_nh, move_group, global_plan, -2);
				 break;
			case 4 : state4Action(_nh, move_group, global_plan, -2);
				 break;
			case 5 : state5Action(_nh, move_group, global_plan, -2);
				 break;
			case 6 : state6Action(_nh, move_group, global_plan, -2);
				 break;
			case 7 : state7Action(_nh, move_group, global_plan, -2);
				 break;
			}//end of executing if.
		}//end of execution conditional
	return actionType;
};//end of stateHandler

//-----------------------------------------------------------------------------------------------------------------------------------//

double EEFplotting(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, double t0, std::string actionType){
		if(state == 4 && actionType == "Executing advance"){ //CHECK THIS STATE NUM AFTER CHANGES
			_nh.getParam("/control/SOT", SOT);
			geometry_msgs::Point EEFxyz = move_group.getCurrentPose().pose.position;
			if(!enablePainting){//perform painting function has been called, delay expended and enablePainting set to false. Allow spraying.	
				if((SOT == 1) && (EEFxyz.z > (2.0+1.0))){
					//_nh.setParam("/control/ballValve", 1);
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


//------------------------------------------------------------------------------------------------------------------------------//

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "control_loop");
	moveit::planning_interface::MoveGroupInterface move_group("all");
	move_group.setMaxAccelerationScalingFactor(1.0);
	move_group.setMaxVelocityScalingFactor(0.9);//0.6
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
	bool remoteState;
	//eulerAngles euler_angles;
	//_nh.getParam("/remoteControlBoom", remoteState);
	//printf("------------------------------------------------------------------------");
	//printf(remoteState ? "primed.\n" : "unprimed.\n");
	spinner.start();
	//const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("all");
//	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	double t0 = ros::Time::now().toSec();
	//double t0_2 = ros::Time::now().toSec();
	std::string actionType;
	_nh.setParam("/control/directionOfSurface", surfaceDirection);
	_nh.getParam("/CAN/virtual", virtualCAN);

//......................................................//
// Define a collision object ROS message.
 	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = move_group.getPlanningFrame();

	// The id of the object is used to identify it.
	collision_object.id = "wall";

	// Define a box to add to the world.
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.01;
	primitive.dimensions[1] = 10.0;
	primitive.dimensions[2] = 7.0;

 	// Define a pose for the box (specified relative to frame_id)
	geometry_msgs::Pose wall_pose;
	wall_pose.orientation.w = 1.0;
	wall_pose.position.x = 3.2;
	wall_pose.position.y = 0.0;
	wall_pose.position.z = 3.5;


  	collision_object.primitives.push_back(primitive);
  	collision_object.primitive_poses.push_back(wall_pose);
  	collision_object.operation = collision_object.ADD;

  	std::vector<moveit_msgs::CollisionObject> collision_objects;
  	collision_objects.push_back(collision_object);

	planning_scene_interface.applyCollisionObjects(collision_objects);

//......................................................//

	while(ros::ok()){
		//plan_subscriber.subscribe();
		std::tuple <int, int> execution_sub_output = execute_subscriber.subscribe();
		executionResult = std::get<0>(execution_sub_output);
		newExecutionResult = std::get<1>(execution_sub_output);
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

#include<hinowa_control/control_subscriber.h>
#include<hinowa_control/plan_result_subscriber.h>
#include<hinowa_control/execute_result_subscriber.h>
#include <fstream>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <hinowa_control/hinowa_state.h>
#include <hinowa_parameters/ctrl_loop_publisher.h>
#include <hinowa_parameters/hwi_subscriber.h>

using namespace std; 

//-----------------------------------------------------------------------------------------------------------------------------------//

void createCartesianPath(moveit::planning_interface::MoveGroupInterface& move_group, control::hinowa_state* hinowa, geometry_msgs::Pose goal_pose){
	std::vector<geometry_msgs::Pose> waypoints;

	waypoints.push_back(goal_pose);

	moveit_msgs::RobotTrajectory trajectory;

	const double jump_threshold = 0; //was 1000
	const double eef_step = 0.01;
	const bool collisions = true;

	ROS_INFO_NAMED("INFO", "Computing plan for painting (Cartesian path) (%.2f%% achieved)", move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, collisions) * 100.0);

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

	hinowa->plan.trajectory_ = trajectory;
}

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
	//TODO:Test whether max avg speed is being exceeded, what is actually being acheived, does this need to be handled. 
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

void storeTrajectoryPlan(moveit_msgs::RobotTrajectory trajectory, std::string current_state){
	std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = trajectory.joint_trajectory.points;

	std::ofstream outfile("/home/josh/workspace/src/hinowacpp/points.csv",std::ios::app);  

	std::vector<int>::size_type size = trajectory_points.size(); 
	std::vector<int>::size_type numJoints = trajectory.joint_trajectory.joint_names.size();

	for(unsigned i = 0; i<size; i++){
		outfile<<current_state<<" - point_index: "<<i<<","<<trajectory_points[i].time_from_start<<","<<"\n";
		for (unsigned j=0; j<numJoints; j++){
			outfile<<trajectory.joint_trajectory.joint_names[j]<<","<<trajectory_points[i].positions[j]<<","<<trajectory_points[i].velocities[j]<<","<<trajectory_points[i].accelerations[j]<<",";
		}
		outfile<<"\n";
	}
	outfile.close();
}

void moveGroupSettings(moveit::planning_interface::MoveGroupInterface& move_group, double planningTime, double maxVelFactor, double maxAccelFactor, double goalTolerance){
	move_group.setPlanningTime(planningTime);
	move_group.setMaxVelocityScalingFactor(maxVelFactor);
	move_group.setMaxAccelerationScalingFactor(maxAccelFactor);
	move_group.setGoalTolerance(goalTolerance);
// the above line calls : setGoalJointTolerance(double tolerance), setGoalOrientationTolerance(double tolerance) and setGoalPositionTolerance(double tolerance).
//if the goal is a joint goal, this is per joint tolerance in radians or meters. NOT IDEAL. For pose targets its a sphere around the goal with radius = tolerance. 
}
//--------------------------------------------------------------------------------------------------------------------------------------------//

void state0Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, control::hinowa_state* hinowa){//track control, execute advances to autolvl

	switch(hinowa->action){
		case  -2:{
			cout<<"State 0.\n"; //finish executing print statement for particular state. 
			break;
		}
		case  -1:{
			cout<<"State 0.\n"; //finish executing print statement for particular state. 
			break;
		}

		case  2:{
			hinowa->setState(1);
			printf("State: %d.\n", hinowa->getState());
			hinowa->active_valve_block = "bottom";
			hinowa->reset_levelling = 1;
			break;
		}
	}
}
	
void state1Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, control::hinowa_state* hinowa){ //auto level, execute for rotate to surface
	switch(hinowa->action){
		case -2:{ 
			cout<<"State 7.\n"; //finish executing print statement for particular state. 
			hinowa->setState(hinowa->getRegressState());
			break;
		}

		case -1:{
			cout<<"State 7.\n"; //finish priming print statement for particular state.
			hinowa->setRegressState(7);
			break;
		}

		case  0:{
			//move_group.setPoseTarget(move_group.getCurrentPose().pose); //this errors, find alternative way to clear plan.
			//move_group.plan(hinowa->plan);
			break;
		}

		case  1:{
			move_group.setPlanningTime(30.0);
			move_group.setStartStateToCurrentState();
			//geometry_msgs::Pose goal_pose = move_group.getCurrentPose().pose;

			//TODO: add removal of collision objects after rotation & after each painting movement as well (to have it updated on the next loop based on newer reading of the target code).
			//TODO: add back in collision object after tuning is completed.
			//hinowa->removeCollisionObjects();
			//if(hinowa->levelled) hinowa->updateCollisionObjects(move_group.getCurrentPose().pose);
			move_group.setJointValueTarget(hinowa->face_surface);//move_group.setJointValueTarget(hinowa->face_surface);
			moveGroupSettings(move_group, 30.0, 0.5, 0.5, 0.001); //test these values with joint targets
			move_group.plan(hinowa->plan);



			if(hinowa->record_data) storeTrajectoryPlan(hinowa->plan.trajectory_, "Rotate to face surface"); //in file hinowacpp/points.csv
			break;
		}

		case  2:{
			if(hinowa->levelled){
				hinowa->active_valve_block = "top";
				ROS_INFO_NAMED("INFO","Execution command %s", (move_group.asyncExecute(hinowa->plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ? "succeeded.\n" : "FAILED.\n");
			}
			else hinowa->setState(1);
			break;
		}
	}
}

void state2Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, control::hinowa_state* hinowa){//face surface, plan to determine whether code alignment is satisfactory. If not, execution will return to state 0, if it is, state advances and plan to move to within painting distance of code is executed. 

	switch(hinowa->action){
		case -2:{
			cout<<"State 1.\n"; //finish executing print statement fo	r particular state.
			//hinowa->active_valve_block = "top"; //dont think this is needed..
			ROS_INFO_NAMED("INFO","Execution command %s", (move_group.asyncExecute(hinowa->plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ? "succeeded.\n" : "FAILED.\n"); 
			break;
		}

		case -1:{
			cout<<"State 1.\n"; //finish priming print statement for particular state.
			hinowa->setRegressState(1);
			moveGroupSettings(move_group, 30.0, 0.5, 0.5, 0.001);
		 	move_group.setJointValueTarget(hinowa->home);
			move_group.plan(hinowa->plan);
			break;
		}

		case  0:{
			move_group.setPoseTarget(move_group.getCurrentPose().pose); //if this errors, find alternative way to clear plan.
			move_group.plan(hinowa->plan);
			break;
		}

		case  1:{
			moveGroupSettings(move_group, 30.0, 0.5, 0.5, 0.001); //relax last value if struggling to find mm tolerance on pose.
			//move_group.setStartStateToCurrentState();

			hinowa->updateWallPoses(move_group.getCurrentPose().pose);
			hinowa->removeCollisionObjects();
			if(hinowa->target_aruco_visible && hinowa->target_aruco_aligned){
				hinowa->updateCollisionObjects(move_group.getCurrentPose().pose);
				printf("Wall distance estimations relative to; Camera: %f, base origin: %f.\n", hinowa->target_code.transform.translation.z, hinowa->approach_code.position.x);
				move_group.setPoseTarget(hinowa->approach_code);
				move_group.plan(hinowa->plan);

				if(hinowa->record_data) storeTrajectoryPlan(hinowa->plan.trajectory_, "Approach Aruco code"); //in file hinowacpp/points.csv
			}

			else{
				hinowa->action = 0;
				hinowa->active_valve_block = "bottom";
				hinowa->setState(0);
				printf("State: %d.\n", hinowa->getState());
			}
			
			break;

		}

		case  2:{
			ROS_INFO_NAMED("INFO","Execution command %s", (move_group.asyncExecute(hinowa->plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ? "succeeded.\n" : "FAILED.\n");
			break;
		}
	}
}

void state3Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, control::hinowa_state* hinowa){//Aruco code approached, execute for move to top
	switch(hinowa->action){
		case -2:{
			cout<<"State 2.\n"; //finish executing print statement for particular state.
			ROS_INFO_NAMED("INFO","Execution command %s", (move_group.asyncExecute(hinowa->plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ? "succeeded.\n" : "FAILED.\n"); 
			break;
		}

		case -1:{
			cout<<"State 2.\n"; //finish priming print statement for particular state.
			hinowa->setRegressState(2);
			//check if any changes to levelling variables is needed to revert to the correct state without having any of the levelling procedure restart again.
			//try adding path constraints to this plan
			moveGroupSettings(move_group, 30.0, 0.5, 0.5, 0.001);
			move_group.setJointValueTarget(hinowa->face_surface);
			move_group.plan(hinowa->plan);
			break;
		}

		case  0:{
			move_group.setPoseTarget(move_group.getCurrentPose().pose); //if this errors, find alternative way to clear plan.
			move_group.plan(hinowa->plan);
			break;
		}

		case  1:{
			moveGroupSettings(move_group, 30.0, 1.0, 0.5, 0.001); //time to plan, vel scale, acc scale, final 6DOF pose tolerance.
	//---// TODO: Steadying Joint 1 while painting - Remove if solved elsewhere, try alternatives if needed. 
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

			createCartesianPath(move_group, hinowa, hinowa->surface_top);

			if(hinowa->record_data) storeTrajectoryPlan(hinowa->plan.trajectory_, "Move to top of surface"); //in file hinowacpp/points.csv

			setAvgCartesianSpeed(hinowa->plan, "end", 0.2);//0.3
			break;
		}

		case  2:{
			ROS_INFO_NAMED("INFO","Execution command %s", (move_group.asyncExecute(hinowa->plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ? "succeeded.\n" : "FAILED.\n");
			break;
		}
	}
}

void state4Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, control::hinowa_state* hinowa){//top of surface state, exec to paint
	switch(hinowa->action){
		case -2:{
			cout<<"State 2.\n"; //finish executing print statement for particular state.
			ROS_INFO_NAMED("INFO","Execution command %s", (move_group.asyncExecute(hinowa->plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ? "succeeded.\n" : "FAILED.\n"); 
			break;
		}

		case -1:{
			cout<<"State 2.\n"; //finish priming print statement for particular state.
			hinowa->setRegressState(2);
			//try adding path constraints to this plan
			moveGroupSettings(move_group, 30.0, 0.5, 0.5, 0.001);
			move_group.setJointValueTarget(hinowa->face_surface);
			move_group.plan(hinowa->plan);
			break;
		}

		case  0:{
			move_group.setPoseTarget(move_group.getCurrentPose().pose); //if this errors, find alternative way to clear plan.
			move_group.plan(hinowa->plan);
			break;
		}

		case  1:{
			moveGroupSettings(move_group, 30.0, 1.0, 0.5, 0.001); //time to plan, vel scale, acc scale, final 6DOF pose tolerance.
			createCartesianPath(move_group, hinowa, hinowa->surface_bottom);

			if(hinowa->record_data) storeTrajectoryPlan(hinowa->plan.trajectory_, "Painting surface"); //in file hinowacpp/points.csv

			setAvgCartesianSpeed(hinowa->plan, "end", 0.3);//0.3
			break;
		}

		case  2:{
			//TODO: test timing balance between these two following calls. Shouldnt start sprying before up to velocity, shouldnt move more thn it has to before starting to paint. Is there a better way to acheive this? 
			hinowa->startPaintingMovement(move_group);
			hinowa->ball_valve = 1;
			printf("Valve open.\n");
			break;
		}
	}
}

void state5Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, control::hinowa_state* hinowa){//painting state, execute to retract
	switch(hinowa->action){
		case -2:{
			cout<<"State 4.\n"; //finish executing print statement for particular state.
			ROS_INFO_NAMED("INFO","Execution command %s", (move_group.asyncExecute(hinowa->plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ? "succeeded.\n" : "FAILED.\n"); 
			break;
		}

		case -1:{
			cout<<"State 4.\n"; //finish priming print statement for particular state.
			hinowa->setRegressState(4);
			moveGroupSettings(move_group, 30.0, 0.5, 0.5, 0.001);
			createCartesianPath(move_group, hinowa, hinowa->surface_top);

			if(hinowa->record_data) storeTrajectoryPlan(hinowa->plan.trajectory_, "Return to top of surface"); //in file hinowacpp/points.csv
			setAvgCartesianSpeed(hinowa->plan, "end", 0.2);//0.3
			break;
		}

		case  0:{
			move_group.setPoseTarget(move_group.getCurrentPose().pose); //if this errors, find alternative way to clear plan.
			move_group.plan(hinowa->plan);
			break;
		}

		case  1:{
			moveGroupSettings(move_group, 30.0, 0.5, 0.5, 0.001); //test these values with joint targets
			move_group.setJointValueTarget(hinowa->face_surface);
			move_group.plan(hinowa->plan);

			if(hinowa->record_data) storeTrajectoryPlan(hinowa->plan.trajectory_, "Retract away from wall"); //in file hinowacpp/points.csv
			break;
		}

		case  2:{
			ROS_INFO_NAMED("INFO","Execution command %s", (move_group.asyncExecute(hinowa->plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ? "succeeded.\n" : "FAILED.\n");
			break;
		}
	}
}

void state6Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, control::hinowa_state* hinowa){//retracted state, execute to unlevel or repeat without unlevelling/relevelling - keeping the same aruco code target ID.
	switch(hinowa->action){
		case -2:{
			cout<<"State 2.\n"; //finish executing print statement for particular state.
			hinowa->setState(hinowa->getRegressState());
			//ROS_INFO_NAMED("INFO","Execution command %s", (move_group.asyncExecute(hinowa->plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ? "succeeded.\n" : "FAILED.\n"); 
			break;
		}

		case -1:{
			cout<<"State 2.\n"; //finish priming print statement for particular state.
			printf("Performing regression of state will keep Target ArUco ID set to: %d, execute if another layer of paint is required in the current position.\n", hinowa->getTargetArucoID());
			hinowa->setRegressState(2);
			//TODO: check if any changes to levelling variables is needed to revert to the correct state without having any of the levelling procedure restart again.
			//try adding path constraints to this plan
			//moveGroupSettings(move_group, 30.0, 0.5, 0.5, 0.001);
			//move_group.setJointValueTarget(hinowa->home);
			//move_group.plan(hinowa->plan);
			break;
		}

		case  0 :{
			move_group.setPoseTarget(move_group.getCurrentPose().pose); //if this errors, find alternative way to clear plan.
			move_group.plan(hinowa->plan);
			break;
		}

		case 1 :{
			printf("Advancement of state will increment Target ArUco ID to: %d, execute if paint application in the current position is satisfactory.\n", hinowa->getTargetArucoID()+1);
			break;
		}

		case  2:{
			hinowa->setState(7);
			hinowa->active_valve_block = "bottom";
			hinowa->setTargetArucoID(hinowa->getTargetArucoID()+1);
			/* TODO: replace this simple implementation of one ultrasonic with a more modular impl. for the new bunch on the EEF.
			double distanceToSurface;
			_nh.getParam("/control/distanceToSurface", distanceToSurface);
			printf("Distance from surface: %fcm.\n", distanceToSurface);
			*/
			break;
		}
	}
}


void state7Action(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, control::hinowa_state* hinowa){//unlevel state, execute for return to 0
	switch(hinowa->action){
		case  2:{
			hinowa->setState(0);
			hinowa->active_valve_block = "bottom";
			//TODO: increment target_aruco_ID - consider current loop to have successfully applied paint.
			break;
		}
	}
}

//-----------------------------------------------------------------------------------------------------------------------------------//

void stateUpdater(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, int executionResult, control::hinowa_state* hinowa){
	cout<<"Action type: "<<hinowa->action<<".\n";	
	printf("Execution result: %d.\n",executionResult);

	if(hinowa->virtualCAN) hinowa->setState(hinowa->getState()+1);
	else{
		if(executionResult == 1){
			if(hinowa->action == 2){
				hinowa->setState(hinowa->getState()+1);
			}
			else if(hinowa->action == -2){ //PS button/regress, TODO: test what states are most useful in practice (set in action functions).
				hinowa->setState(hinowa->getRegressState());
			}
		}
		else{
//TODO: MAKE THIS RELIABLY CLEAR FAILED TRAJ. DESIRED POSES.
/*
			ros::Duration(1.0).sleep(); // sleep for a second
			moveGroupSettings(move_group, 30.0, 1.0, 1.0, 0.001);
			move_group.setPoseTarget(move_group.getCurrentPose().pose);
			move_group.plan(hinowa->plan);
			move_group.asyncExecute(hinowa->plan);
*/
		}

	}
	printf("State: %d.\n", hinowa->getState());
}
void stateHandler(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, control::hinowa_state* hinowa, int updated)
{
	if(updated){
		switch(hinowa->getState()){
			case 0 : state0Action(_nh, move_group, hinowa);
				 break;
			case 1 : state1Action(_nh, move_group, hinowa);
				 break;
			case 2 : state2Action(_nh, move_group, hinowa);
				 break;
			case 3 : state3Action(_nh, move_group, hinowa);
				 break;
			case 4 : state4Action(_nh, move_group, hinowa);
				 break;
			case 5 : state5Action(_nh, move_group, hinowa);
				 break;
			case 6 : state6Action(_nh, move_group, hinowa);
				 break;
			case 7 : state7Action(_nh, move_group, hinowa);
				 break;
			
		}
	}
};//end of stateHandler

//-----------------------------------------------------------------------------------------------------------------------------------//
void paintHandler(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, control::hinowa_state* hinowa){
	
	if(hinowa->getState() == 4 && hinowa->action == 2){
//TODO: sort out this function
		if((hinowa->state_of_trajectory == 1) && (hinowa->EEFxyz.z > (2.0+1.0))){ //MAKE THIS DYNAMICALLY DETERMINED BASED ON VELOCITY AND THE TIME IT TAKES FOR THE VALVE TO CLOSE. 2.0M IS ABS MIN HEIGHT FOR MACHINE. 
			hinowa->ball_valve = 1; //UNCOMMENT FOR AUTO VALVE CONTROL
			printf("Valve open.\n");
		}
		else{
			hinowa->ball_valve = 0;
			if(!hinowa->ball_valve && hinowa->prev_ball_valve)  
				printf("Valve closed. SOT: %d, EEF: %f.\n", hinowa->state_of_trajectory, hinowa->EEFxyz.z);
		}
		hinowa->prev_ball_valve = hinowa->ball_valve;
	}
}

void EEFplotting(ros::NodeHandle _nh, moveit::planning_interface::MoveGroupInterface& move_group, double t0, control::hinowa_state* hinowa){
			//int SOT;
			//_nh.getParam("/control/SOT", SOT); //change this to a topic
			if((hinowa->EEFxyz.x != hinowa->EEFxyz_prev.x) || (hinowa->EEFxyz.y != hinowa->EEFxyz_prev.y) || (hinowa->EEFxyz.z != hinowa->EEFxyz_prev.z)){
				hinowa->Te = ros::Time::now().toSec()-t0;
				//printf("End effector position, X: %f, Y: %f, Z: %f, t: %f.\n", EEFxyz.x, EEFxyz.y, EEFxyz.z, t);
				std::ofstream outfile("/home/josh/workspace/src/hinowacpp/EEFposition.csv",std::ios::app);
				outfile<<hinowa->EEFxyz.x<<","<<hinowa->EEFxyz.y<<","<<hinowa->EEFxyz.z<<","<<hinowa->Te;
				//z velocity
				double Vz = ((hinowa->EEFxyz.z)-(hinowa->EEFxyz_prev.z))/((hinowa->Te)-(hinowa->Te_prev));
				outfile<<","<<Vz;
	
				outfile<<"\n";
				outfile.close();
				hinowa->EEFxyz_prev.x = hinowa->EEFxyz.x; hinowa->EEFxyz_prev.y = hinowa->EEFxyz.y; hinowa->EEFxyz_prev.z = hinowa->EEFxyz.z; hinowa->Te_prev = hinowa->Te;
			}
}

//------------------------------------------------------------------------------------------------------------------------------//

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "control_loop");
	moveit::planning_interface::MoveGroupInterface move_group("all");

	ros::NodeHandle _nh;
	ros::AsyncSpinner spinner(4);
	control::CONTROL_SUBSCRIBER control_subscriber = control::CONTROL_SUBSCRIBER(_nh);
	control::PLAN_RESULT_SUBSCRIBER plan_subscriber = control::PLAN_RESULT_SUBSCRIBER(_nh);
	control::EXECUTE_RESULT_SUBSCRIBER execute_subscriber = control::EXECUTE_RESULT_SUBSCRIBER(_nh);
	control::hinowa_state hinowa(_nh);
	hinowa_parameters::ctrl_loop_publisher control_loop_pub(_nh);
	hinowa_parameters::hwi_subscriber hwi_sub(_nh);
//......................................................//
	double t0 = ros::Time::now().toSec();
	int executionFlag = 0; int executionResult = 0;
	std::tuple<int,int,int> controller_output; int Select = 0; int Start = 0; int PS = 0; //controller buttons
	double loop_hz;
	_nh.getParam("/hinowa/hardware_interface/loop_hz", loop_hz);
	ros::Rate rate(loop_hz);
	spinner.start();
//......................................................//
	while(ros::ok()){
	double t1 = ros::Time::now().toSec();

		hwi_sub.subscribe(); 

		hinowa.levelled = hwi_sub.levelled;
		hinowa.state_of_trajectory = hwi_sub.state_of_trajectory;
		hinowa.loop_dt = 1/hwi_sub.loop_hz;
		//printf("HWI params - levelled: %d, SOT: %d, loop_dt: %f.\n", hinowa.levelled, hinowa.state_of_trajectory, hinowa.loop_dt); 

		std::tuple <int, int> execution_sub_output = execute_subscriber.subscribe();
		executionResult = std::get<0>(execution_sub_output);
		executionFlag = std::get<1>(execution_sub_output);
		hinowa.planResult = plan_subscriber.subscribe();
		if(hinowa.planResult != 1) hinowa.action = 0;

		controller_output = control_subscriber.subscribe();
		Select = get<0>(controller_output);
		Start = get<1>(controller_output);
		PS = get<2>(controller_output);

		if(Select || Start || PS) stateHandler(_nh, move_group, &hinowa, hinowa.updateAction(Select, Start, PS));

		hinowa.EEFxyz = move_group.getCurrentPose().pose.position;
		paintHandler(_nh, move_group, &hinowa);
		if(hinowa.record_data) EEFplotting(_nh, move_group, t0, &hinowa);

		if(executionFlag) stateUpdater(_nh, move_group, executionResult, &hinowa);

		control_loop_pub.addData(hinowa.getState(), hinowa.getTargetArucoID(), hinowa.active_valve_block, hinowa.ball_valve, hinowa.reset_levelling);
		control_loop_pub.publish();
		if(hinowa.reset_levelling) hinowa.reset_levelling = 0; //send only once so it doesn't infinitely reset.

		//if(hinowa.getTargetArucoID > hinowa.final_aruco_ID) //Decide what to do here; kill control loop? all code? lock to state 0? make sure the paint loop cant be accessed but still allow for manually driving the lift around a corner/back onto the trailer etc.

		rate.sleep();
	}
	ros::waitForShutdown();
	return 0;
}

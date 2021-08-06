#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <nodelet/nodelet.h>

#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf/transform_datatypes.h>
/*
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
//#include <moveit/core/transforms/transforms.h>	// Eigen transforms
*/

/*
void paintWall()
{
	//make a function call for the cartesian pathing developed below
	ros::NodeHandle nodeHandle;
}; // End of function paintWall
*/

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_robot");
	//print out notifier that this file has been launched.
	ROS_INFO("Control node successfully launched.");
	//MoveRobot mv; //call to class above (commented out to follow tutorial code)
	ros::NodeHandle nodeHandle; //defining a node handle (not sure what this does yet)
	ros::AsyncSpinner spinner(1); //Current spinner settings runs this main function once - address this if this needs to change
	spinner.start();
//-----------------------------------------------------------------------------------------
	//define move_group for all joints
	moveit::planning_interface::MoveGroupInterface move_group("all");	
	//define arm_move_group
//	moveit::planning_interface::MoveGroupInterface arm_move_group("arm");

	//add/remove collision objects using this
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	//Using raw pointer for the planning group
	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("all");
//-----------------------------------------------------------------------------------------
/*
	//loading in wall...
	// Define a collision object ROS message.
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = move_group.getPlanningFrame();
//	moveit_msgs::CollisionObject collision_object_arm;
//	collision_object_arm.header.frame_id = arm_move_group.getPlanningFrame();
	moveit_msgs::CollisionObject floor;
	floor.header.frame_id = move_group.getPlanningFrame();


	// The id of the object is used to identify it.
	collision_object.id = "wall";
//	collision_object_arm.id = "wall";
 	floor.id = "asphalt_plane";

	// Define a box to add to the world.
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.25;
	primitive.dimensions[1] = 6;
	primitive.dimensions[2] = 12;


	// Define a pose for the box (specified relative to frame_id)
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1;
	box_pose.position.x = -3.125;
	box_pose.position.y = 0;
	box_pose.position.z = 6;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	//asphalt_plane
	//shape_msgs::SolidPrimitive floor;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 10;
	primitive.dimensions[1] = 10;
	primitive.dimensions[2] = 0.01;
	//asphalt_plane
	geometry_msgs::Pose floor_pose;
	floor_pose.orientation.w = 1;
	floor_pose.position.x = 0;
	floor_pose.position.y = 0;
	floor_pose.position.z = 0;

	floor.primitives.push_back(primitive);
	floor.primitive_poses.push_back(floor_pose);
	floor.operation = collision_object.ADD;
	
//	collision_object_arm.primitives.push_back(primitive);
//	collision_object_arm.primitive_poses.push_back(box_pose);
//	collision_object_arm.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);
//	collision_objects.push_back(collision_object_arm);
	collision_objects.push_back(floor);

	//ROS_INFO("Add an object into the world");
	planning_scene_interface.addCollisionObjects(collision_objects);
*/
//-------------------------------------------------------------------------------------
	//setting up visualtoolsgui to use next button for stepping through code
	namespace rvt = rviz_visual_tools;
	
	moveit_visual_tools::MoveItVisualTools visual_tools("world");
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();
 	visual_tools.trigger();
//-------------------------------------------------------------------------------------	

	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to allow movement");	
	//pointer to reference robots state
	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	// Next get the current set of joint values for the group.
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	for(int i=0; i<5; i++){
		printf("%lf",joint_group_positions[i]);	
	}

  	// extend wrist joint so it isn't folded back into the robot
	joint_group_positions[5] = 0.5;  // radians
	//joint_group_positions[1] = 0.7071;
	move_group.setJointValueTarget(joint_group_positions);
	move_group.move();
/*
//-------------------------------------------------------------------------------------	
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to allow movement");	
	//pointer to reference robots state
	moveit::core::RobotStatePtr current_state2 = move_group.getCurrentState();
	// Next get the current set of joint values for the group.
	std::vector<double> joint_group_positions2;
	current_state2->copyJointGroupPositions(joint_model_group, joint_group_positions2);
	for(int i=0; i<5; i++){
		printf("%lf",joint_group_positions2[i]);	
	}

  	// extend wrist joint so it isn't folded back into the robot
	joint_group_positions[5] = 1.57071;  // radians
	//joint_group_positions[1] = 0.7071;
	move_group.setJointValueTarget(joint_group_positions2);
	move_group.move();
*/
//--------------------------moving to start point---------------------------------------
/*
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
	geometry_msgs::Pose first_pose = move_group.getCurrentPose().pose;

        //tf::Quaternion q;
        //q.setEuler(3.14159,0,3.14159);

        //first_pose.orientation.x = q.x();
        //first_pose.orientation.y = q.y();
        //first_pose.orientation.z = q.z();
        //first_pose.orientation.w = q.w();
	
	first_pose.position.x = 0;
 	first_pose.position.y = 0;
	first_pose.position.z = 5;
	move_group.setPoseTarget(first_pose);	

	moveit::planning_interface::MoveGroupInterface::Plan first_plan;
	move_group.move();
	bool success1 = (move_group.plan(first_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial","Visualizing plan 1 (pose goal) %s", success1 ? "succeeded" : "FAILED");
*/
//----------------------------------------------------------------------------------------------------
/*
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "end_mount";
	ocm.header.frame_id = "base_link";
	ocm.orientation.x = 0.002841;
	ocm.orientation.y = -0.000047;
	ocm.orientation.z = 0.000089;
	ocm.orientation.w = 1;
	//ocm.absolute_x_axis_tolerance = 3.14159;
	//ocm.absolute_y_axis_tolerance = 3.14159;
	//ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	move_group.setPathConstraints(test_constraints);
/*	
/*
	geometry_msgs::Pose next_pose;

        tf::Quaternion q_next;
        q_next.setEuler(0,1.57071,3.14159);

        next_pose.orientation.x = q_next.x();
        next_pose.orientation.y = q_next.y();
        next_pose.orientation.z = q_next.z();
        next_pose.orientation.w = q_next.w();
	
	next_pose.position.x = 0.53973110515;
 	next_pose.position.y = 1.10122342169;
	next_pose.position.z = 4.49085384405;
	move_group.setPoseTarget(next_pose);	
*/
/*
//	move_group.setPlanningTime(120.0);
	move_group.setNamedTarget("max");
	moveit::planning_interface::MoveGroupInterface::Plan next_plan;
	move_group.move();
	bool success2 = (move_group.plan(next_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial","Visualizing plan 1 (pose goal) %s", success2 ? "succeeded" : "FAILED");
*/
//------------------------------------------------------------------------------------------------------------
//find out why the percentage changes with seemingly meaningless code changes
//joint constraint for boom - faster movement when not necessary
/*	
	moveit_msgs::JointConstraint jc;
	jc.joint_name = "joint_7";  
	jc.position = 0.0;
	jc.tolerance_above = 0.1;
	jc.tolerance_below = 0;
	jc.weight = 1.0; 
	moveit_msgs::Constraints path_constraints;
	path_constraints.joint_constraints.push_back(jc);
	move_group.setPathConstraints(path_constraints);
*/	
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");	
	move_group.setNamedTarget("extended");
	moveit::planning_interface::MoveGroupInterface::Plan first_plan;
	move_group.move();
	bool success3 = (move_group.plan(first_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial","Visualizing plan 1 (pose goal) %s", success3 ? "succeeded" : "FAILED");

	geometry_msgs::Pose end_pose = move_group.getCurrentPose().pose;	
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");	
	move_group.setNamedTarget("max");
	moveit::planning_interface::MoveGroupInterface::Plan next_plan;
	move_group.move();
	bool success2 = (move_group.plan(next_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial","Visualizing plan 1 (pose goal) %s", success2 ? "succeeded" : "FAILED");

	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
	std::vector<geometry_msgs::Pose> waypoints; //for lift only

	geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
	waypoints.push_back(start_pose);
	
	//geometry_msgs::Pose target_pose = start_pose;
	//target_pose.position.x -= 1;
	//target_pose.position.y -= 1;
	//target_pose.position.z += 1;
	waypoints.push_back(end_pose);  // down

	//target_pose.position.z += 1;
	//target_pose.position.x += 1;
	//waypoints.push_back(target_pose);  // down
	//all_waypoints.push_back(target_pose);

/*
	target_pose.position.x += 0.5;
	waypoints.push_back(target_pose); //right

	target_pose.position.z += 0.4;
	waypoints.push_back(target_pose); //up
*/

	
	// Cartesian motions are frequently needed to be slower for actions such as approach and retreat
 	// grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
 	// of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
	move_group.setMaxVelocityScalingFactor(1);//used to be 0.5
 	// We want the Cartesian path to be interpolated at a resolution of 1 cm
 	// which is why we will specify 0.01 as the max step in Cartesian
 	// translation.  We will specify the jump threshold as 0.0, effectively disabling it.
 	// Warning - disabling the jump threshold while operating real hardware can cause
 	// large unpredictable motions of redundant joints and could be a safety issue
 	moveit_msgs::RobotTrajectory trajectory;
	//moveit_msgs::RobotTrajectory arm_trajectory;
 	
	move_group.setPlanningTime(60.0);

 	const double jump_threshold = 0; //was 1000
 	const double eef_step = 0.01;
	const bool collisions = true;
 	double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, collisions);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
	
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

  	// Finally plan and execute the trajectory
	moveit::planning_interface::MoveGroupInterface::Plan plan;
  	plan.trajectory_ = trajectory;
  	ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);   
  	sleep(15.0);
  	move_group.execute(plan); //uncomment to execute

//-------------------------------------------------------------------------------------------------------------
/*	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
	std::vector<geometry_msgs::Pose> arm_waypoints; //for arm only
	std::vector<geometry_msgs::Pose> all_waypoints; //for all joints
	
	arm_waypoints.push_back(start_pose);
	geometry_msgs::Pose target_pose = start_pose;

	target_pose.position.z -= 0.4;
	arm_waypoints.push_back(target_pose);  // down
	//all_waypoints.push_back(target_pose);

	target_pose.position.x += 0.25;
	arm_waypoints.push_back(target_pose); //right

	target_pose.position.z += 0.4;
	arm_waypoints.push_back(target_pose); //up

	
	// Cartesian motions are frequently needed to be slower for actions such as approach and retreat
 	// grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
 	// of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
 	arm_move_group.setMaxVelocityScalingFactor(0.5);
	move_group.setMaxVelocityScalingFactor(0.5);
 	// We want the Cartesian path to be interpolated at a resolution of 1 cm
 	// which is why we will specify 0.01 as the max step in Cartesian
 	// translation.  We will specify the jump threshold as 0.0, effectively disabling it.
 	// Warning - disabling the jump threshold while operating real hardware can cause
 	// large unpredictable motions of redundant joints and could be a safety issue
 	moveit_msgs::RobotTrajectory trajectory;
	//moveit_msgs::RobotTrajectory arm_trajectory;
 	
	//arm_move_group.setPlanningTime(60.0);

 	const double jump_threshold = 0.0;
 	const double eef_step = 0.01;
	const bool collisions = true;
 	double arm_fraction = arm_move_group.computeCartesianPath(arm_waypoints, eef_step, jump_threshold, trajectory, collisions);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", arm_fraction * 100.0);
	
	// The trajectory needs to be modified so it will include velocities as well.
  	// First to create a RobotTrajectory object
  	robot_trajectory::RobotTrajectory rt(arm_move_group.getCurrentState()->getRobotModel(), "arm");

  	// Second get a RobotTrajectory from trajectory
  	rt.setRobotTrajectoryMsg(*arm_move_group.getCurrentState(), trajectory);
 
  	// Thrid create a IterativeParabolicTimeParameterization object
  	trajectory_processing::IterativeParabolicTimeParameterization iptp;

  	// Fourth compute computeTimeStamps
  	success = iptp.computeTimeStamps(rt);
  	ROS_INFO("Computed time stamp %s",success?"SUCCEEDED":"FAILED");

  	// Get RobotTrajectory_msg from RobotTrajectory
  	rt.getRobotTrajectoryMsg(trajectory);

  	// Finally plan and execute the trajectory
	moveit::planning_interface::MoveGroupInterface::Plan plan;
  	plan.trajectory_ = trajectory;
  	ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",arm_fraction * 100.0);   
  	sleep(5.0);
  	arm_move_group.execute(plan);
*/
  /*	
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
	visual_tools.trigger();	
	
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
*/
/*
	double fraction = move_group.computeCartesianPath(all_waypoints, eef_step, jump_threshold, trajectory, collisions);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
*/
//-------------------------------------------------------------------------------------------------------------
	ros::shutdown();
	return 0;
}

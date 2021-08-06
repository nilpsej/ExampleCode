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
//#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf/transform_datatypes.h>
/*
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
//#include <moveit/core/transforms/transforms.h>	// Eigen transforms
*/

#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>

int value;

void storeData(const sensor_msgs::Joy::ConstPtr& joy)
{
	value = joy->buttons[9];
	if(value) ROS_INFO("Value: %d.\n", value);
}
	

void paintWall()
{
	//make a function call for the cartesian pathing developed below
	//ros::NodeHandle nodeHandle;
}; // End of function paintWall


int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_loop");
	moveit::planning_interface::MoveGroupInterface move_group("all");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	//const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("all");
	
	ros::NodeHandle _nh;
	//ros::Publisher chatter_pub = _nh.advertise<std_msgs::Int32>("chatter", 1000);
	ros::Subscriber sub = _nh.subscribe<sensor_msgs::Joy>("/joy", 1000, storeData);
	ros::AsyncSpinner spinner(4); //Current spinner settings runs this main function once - address this if this needs to change
	spinner.start();
	while(ros::ok()){
		//read(_nh);
		/*
		std_msgs::Int32 msg;
		msg.data = value;
		chatter_pub.publish(msg);
		*/
		//printf("Value: %d.\n", value);
	}
	ros::waitForShutdown();
	//ros::spin();
	return 0;
	
/*
	//positive z is upward in world frame, positive y is to the right in world frame from start view
	visual_tools.prompt("Press 'next' to move to lower waypoint");	
	geometry_msgs::Pose pose_one = move_group.getCurrentPose().pose;
	pose_one.position.y += 1.25; 
	pose_one.position.z += 0.5;
	move_group.setPoseTarget(pose_one);
	move_group.setGoalPositionTolerance(0.05);
	move_group.setPlanningTime(60.0);
	moveit::planning_interface::MoveGroupInterface::Plan plan_one;
	move_group.move();
	bool success_one = (move_group.plan(plan_one) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial","Visualizing plan 1 (pose goal) %s", success_one ? "succeeded" : "FAILED");

	visual_tools.prompt("Press 'next' to move to upper waypoint");	
	geometry_msgs::Pose pose_two = move_group.getCurrentPose().pose;
	pose_two.position.z += 4; 
	move_group.setPoseTarget(pose_two);
	move_group.setGoalPositionTolerance(0.05);
	move_group.setPlanningTime(60.0);
	moveit::planning_interface::MoveGroupInterface::Plan plan_two;
	move_group.move();
	bool success_two = (move_group.plan(plan_two) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial","Visualizing plan 1 (pose goal) %s", success_two ? "succeeded" : "FAILED");

	visual_tools.prompt("Press 'next' to execute planar movement");	

	std::vector<geometry_msgs::Pose> waypoints; //for lift only
	waypoints.push_back(pose_two);
	waypoints.push_back(pose_one);  // down

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
	moveit::planning_interface::MoveItErrorCode execute;  	
	execute = move_group.execute(plan); //uncomment to execute
	if(execute == true) printf("Movement complete\n");
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
/*
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
	moveit::planning_interface::MoveItErrorCode execute;  	
	execute = move_group.execute(plan); //uncomment to execute
	if(execute == true) printf("Movement complete\n");
*/
//-------------------------------------------------------------------------------------------------------------
//Commented out duplicate of original cartesian motion sim testing removed from here.
//-------------------------------------------------------------------------------------------------------------
	//ros::waitForShutdown(); //use this
	//ros::spin();
	///return 0;
}

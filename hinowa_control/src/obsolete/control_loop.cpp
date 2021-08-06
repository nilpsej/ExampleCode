#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <nodelet/nodelet.h>

#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include <sstream>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
//#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
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
	ros::init(argc, argv, "control_loop");
	//print out notifier that this file has been launched.
	ROS_INFO("Control node successfully launched.");
	//MoveRobot mv; //call to class above (commented out to follow tutorial code)
	ros::NodeHandle nodeHandle; //defining a node handle (not sure what this does yet)
	ros::AsyncSpinner spinner(1); //Current spinner settings runs this main function once - address this if this needs to change
	spinner.start();
//-----------------------------------------------------------------------------------------
	//define move_group for all joints
	moveit::planning_interface::MoveGroupInterface move_group("all");
	moveit::planning_interface::MoveGroupInterface::Plan global_plan;	
	//define arm_move_group
//	moveit::planning_interface::MoveGroupInterface arm_move_group("arm");

	//add/remove collision objects using this
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	//Using raw pointer for the planning group
	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("all");
//-----------------------------------------------------------------------------------------
//Collision code from testcode file removed from here.
//-------------------------------------------------------------------------------------
	//setting up visualtoolsgui to use next button for stepping through code
	//remove eventually - just for ease of use currently	
	namespace rvt = rviz_visual_tools;
	
	//visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/moveit_visual_tools"));

	moveit_visual_tools::MoveItVisualTools visual_tools("world");
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();
 	visual_tools.trigger();

//-------------------------------------------------------------------------------------	
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to allow movement");	
	//taken from actual control loop

		//positive z is upward in world frame, positive y is to the left(?) in world frame from start view
		geometry_msgs::Pose pose = move_group.getCurrentPose().pose;
		printf("Positions - X: %f, Y: %f, Z: %f.\n", pose.position.x, pose.position.y, pose.position.z);
		printf("Orientations - X: %f, Y: %f, Z: %f, W: %f.\n", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

		pose.position.x = 0.0; //figure out these values using the lift with real CAN
		pose.position.y = 3.5; 
		pose.position.z = 6;
		pose.orientation.x = 0.0000000;
		pose.orientation.y = 0.0000000;
		pose.orientation.z = 0.0000000;
		pose.orientation.w = 1;
		//move_group.setPositionTarget(pose.position.x, pose.position.y, pose.position.z, "end");
		move_group.setPoseTarget(pose);
		move_group.setPlanningTime (60.0);
		//move_group.setGoalTolerance(0.1);
		bool result = (move_group.plan(global_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("INFO","Visualizing plan 1 (pose goal) %s", result ? "succeeded" : "FAILED");

		visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to allow movement");
		move_group.asyncExecute(global_plan);

		visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to allow movement");
//----------------------------------------------------------------------------------------------------
		//plan cartesian movement
		geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose; //record start pose (top position).
		std::vector<geometry_msgs::Pose> waypoints;
		

		geometry_msgs::Pose final_pose = start_pose; //create copy of start_pose.
		//make changes required to form desired final pose.
		final_pose.position.z = 1; //set desired final z to 1m.
		waypoints.push_back(start_pose); //add to waypoint list
		waypoints.push_back(final_pose);
		
		move_group.setMaxVelocityScalingFactor(0.1);
		moveit_msgs::RobotTrajectory trajectory;
		
		const double jump_threshold = 1000; //was 1000
 		const double eef_step = 0.30;
		const bool collisions = true;

		double success_fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, collisions);
		ROS_INFO_NAMED("INFO", "Visualizing plan for painting (Cartesian path) (%.2f%% achieved)", success_fraction * 100.0);
		
		visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to allow movement");
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
//
		std::stringstream ss;
         	std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajectory_points;  

         	std::vector<int>::size_type size1 = trajectory.joint_trajectory.points.size();  

		for (unsigned i=0; i<size1; i++){  

             		ss << "point_index: " << i << "\n" 
			<< "names: "  
                	<< "[" << trajectory.joint_trajectory.joint_names[0]  
                	<< "," << trajectory.joint_trajectory.joint_names[1]  
                	<< "," << trajectory.joint_trajectory.joint_names[2]  
                	<< "," << trajectory.joint_trajectory.joint_names[3]  
                	<< "," << trajectory.joint_trajectory.joint_names[4]  
                	<< "," << trajectory.joint_trajectory.joint_names[5]  
                	<< "]" << "\n"
               		<< "positions: "  
                	<< "[" << trajectory.joint_trajectory.points[i].positions[0]  
                	<< "," << trajectory.joint_trajectory.points[i].positions[1]  
                	<< "," << trajectory.joint_trajectory.points[i].positions[2]  
                	<< "," << trajectory.joint_trajectory.points[i].positions[3]  
                	<< "," << trajectory.joint_trajectory.points[i].positions[4]  
                	<< "," << trajectory.joint_trajectory.points[i].positions[5]  
                	<< "]" << "\n"
			<< "velocities: "  
                	<< "[" << trajectory.joint_trajectory.points[i].velocities[0]  
                	<< "," << trajectory.joint_trajectory.points[i].velocities[1]  
                	<< "," << trajectory.joint_trajectory.points[i].velocities[2]  
                	<< "," << trajectory.joint_trajectory.points[i].velocities[3]  
                	<< "," << trajectory.joint_trajectory.points[i].velocities[4]  
                	<< "," << trajectory.joint_trajectory.points[i].velocities[5]  
                	<< "]" << "\n"
			<< "accelerations: "  
                	<< "[" << trajectory.joint_trajectory.points[i].accelerations[0]  
                	<< "," << trajectory.joint_trajectory.points[i].accelerations[1]  
                	<< "," << trajectory.joint_trajectory.points[i].accelerations[2]  
                	<< "," << trajectory.joint_trajectory.points[i].accelerations[3]  
                	<< "," << trajectory.joint_trajectory.points[i].accelerations[4]  
                	<< "," << trajectory.joint_trajectory.points[i].accelerations[5]  
                	<< "]" << "\n"; 
        	}  
		std::ofstream outfile("/home/josh/workspace/points.txt",std::ios::app);
       		if(!outfile.is_open()){
                            ROS_INFO("open failed");
                }
          	else{

             		//outfile<<"trajectory"<<count<<endl;
//              		outfile<<trajectory.multi_dof_joint_trajectory.joint_names[0]<<"\n";
              		outfile<<ss.str()<<"\n";
              		outfile.close();

           	} 
//
		//move_group.plan(global_plan);

		//visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to allow movement");
		//execute cartesian movement
		moveit::planning_interface::MoveItErrorCode execute_outcome;  	
		execute_outcome = move_group.execute(global_plan); //uncomment to execute
		//if(execute_outcome == true) printf("Movement complete\n");
		//turn on paint sprayers can_.writeData(0x60, -1); //sprayers use first bit recognition, must be a negative value!


//-------------------------------------------------------------------------------------------------------------
//Commented out duplicate of original cartesian motion sim testing removed from here.
//-------------------------------------------------------------------------------------------------------------
	ros::waitForShutdown();
	//ros::spin();
	return 0;
}

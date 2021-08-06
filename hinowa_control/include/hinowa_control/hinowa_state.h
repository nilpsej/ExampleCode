#ifndef HINOWA__STATE_H
#define HINOWA__STATE_H

//#include <iostream>
//#include <ros/console.h>
//#include <nodelet/nodelet.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Float32MultiArray.h>
//#include "std_msgs/String.h"
#include <ros/ros.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <camera_tf2/aruco_subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define num_states 8

namespace control
{
	class hinowa_state
	{
		private:

			ros::NodeHandle nh; //node handler

			
			double paint_delay = 1.5; //time to delay movement for painting to allow valve to open

			const std::string stateInfo[num_states] = {"Remote track and stabiliser control", "Autonomous levelling", "Rotate to face wall", "Aruco code detection", "Move to top of surface", "Apply paint to surface", "Retract from wall", "Autonomous unlevelling"};
			
			std::string surface_direction;
			double surface_distance = 0.0;
			double surface_top_height = 0.0;
			double surface_bottom_height = 0.0;
			double painting_distance = 0.0;

			int state = 0; // [0-7], refer to below for description of each.
			int regress_state = 0;
			int target_aruco_ID = 0;
			//int SOT; //state of trajectory execution, is it in progress, failed etc


		public:
			hinowa_state();
			hinowa_state(ros::NodeHandle nh);
			~hinowa_state();
			void definePoses();
			void removeCollisionObjects();
			void updateCollisionObjects(geometry_msgs::Pose current_pose);
			bool targetArucoAligned();
			void updateWallPoses(geometry_msgs::Pose current_pose);
			int updateAction(int SELECT, int START, int PS);
			int getState();
			void setState(int new_state);
			int getRegressState();
			void setRegressState(int regress_state);
			int getTargetArucoID();
			void setTargetArucoID(int ID);
			void startPaintingMovement(moveit::planning_interface::MoveGroupInterface& move_group); //painting delay

			moveit::planning_interface::MoveGroupInterface::Plan plan; //plan to be executed at each state change
			moveit::planning_interface::PlanningSceneInterface planning_scene_interface; //planning scene for collisions
			moveit_msgs::CollisionObject wall_collision_object;
			moveit_msgs::CollisionObject front_limit_object;
			moveit_msgs::CollisionObject side_limit_object;
			double arc_limit;

			int record_data = 0; //prints out to files for EEF position and trajectory plan
			int action = 0; //-2: Execute home, -1: prime home, 0: unprimed, 1: prime advance, 2: execute advance.
			int update = 0;
			int planResult = 1;
			bool virtualCAN = false;
			int paint = 0; //T/F for painting
			
			geometry_msgs::Point EEFxyz;
			geometry_msgs::Point EEFxyz_prev; //same as above..
			double Te = 0; //T end effector = current T - T0. For plotting.
			double Te_prev = 0; //T end effector previous = (current T-loop_time) - T0. For plotting.

			//geometry_msgs::Pose home;
			//geometry_msgs::Pose face_surface;
			std::vector<double> home;
			std::vector<double> face_surface;
			std::vector<double> face_surface2;
			geometry_msgs::Pose approach_code;
			geometry_msgs::Pose surface_top;
			geometry_msgs::Pose surface_bottom;
			geometry_msgs::TransformStamped target_code;

			camera_tf2::ARUCO_SUBSCRIBER aruco_sub = camera_tf2::ARUCO_SUBSCRIBER(nh);
			std::tuple<std::vector<geometry_msgs::TransformStamped>, std::vector<geometry_msgs::TransformStamped>, int> aruco_tuple;
			std::vector<geometry_msgs::TransformStamped> aruco_anchor;
			std::vector<geometry_msgs::TransformStamped> aruco_camera;
			int num_markers = 0;
			double wall_x = 0.0;
			double painting_x = 0.0;
			
			int levelled = 0;
			int state_of_trajectory = 0;
			double loop_dt = 0;

			int ball_valve = 0; int prev_ball_valve = 0;
			std::string active_valve_block = "bottom";
			int reset_levelling = 0;
			int final_aruco_ID = 0;
			double T_aligned; //pulled from hinowa_control/config/control.yaml
			double T_parallel; //pulled from hinowa_control/config/control.yaml

			bool target_aruco_visible = false;
			bool target_aruco_aligned = false;
			bool update_wall = false;
	};


}

#endif

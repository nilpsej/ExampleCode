//#include <ros/callback_queue.h>
//#include <ros/spinner.h>
/*
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
*/
#include "hinowa_control/hinowa_state.h"

namespace control
{

	hinowa_state::hinowa_state()
	{

	}

	hinowa_state::hinowa_state(ros::NodeHandle nh)
	{
		this->nh = nh;
		nh.getParam("/CAN/virtual", virtualCAN);
		nh.getParam("/surface/painting_distance", painting_distance);
		nh.getParam("/surface/direction", surface_direction);
		nh.getParam("/surface/height", surface_top_height);
		nh.getParam("/surface/stopping_height", surface_bottom_height); //height to stop painting at.
		nh.getParam("/aruco/initial_ID", target_aruco_ID);
		nh.getParam("/aruco/final_ID", final_aruco_ID);
		nh.getParam("/data/record", record_data);
		nh.getParam("/aruco/translation_align_tolerance", T_aligned);
		nh.getParam("/aruco/rotation_align_tolerance", T_parallel);
		if(surface_direction != "left" && surface_direction != "right")
			throw std::runtime_error("Invalid surface Direction. Enter either left or right in /hinowa_control/control.yaml.\n");
		definePoses();
		aruco_tuple = aruco_sub.subscribe();
		wall_collision_object.header.frame_id = "anchor";
		wall_collision_object.id = "wall";
		front_limit_object.header.frame_id = "anchor";
		front_limit_object.id = "rotate_limit_front";
		side_limit_object.header.frame_id = "anchor";
		side_limit_object.id = "rotate_limit_side";
	}

	hinowa_state::~hinowa_state()
	{
	}

	void hinowa_state::definePoses(){
// j1, j4, j5, j6, j7, j8, j9, j_end_bot, j_end_middle
		home = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //why does this have a 0.01? is it for replanning errors?
		if(surface_direction == "left"){
			//face_surface = {1.57071, -0.8, 0.8, 0.0, 0.0, 0.0, -0.0, -0.0, 0.0};
			//face_surface2 = {1.57071, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0};
			face_surface = {1.57071, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		}
		else if(surface_direction == "right"){
			//face_surface = {-1.57071, -0.8, 0.8, 0.0, 0.0, 0.0, -0.0, -0.0, 0.0};
			//face_surface2 = {-1.57071, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			face_surface = {-1.57071, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		}
	}

	void hinowa_state::removeCollisionObjects(){
		planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames(false));
	}

	void hinowa_state::updateCollisionObjects(geometry_msgs::Pose current_pose){
		std::vector<moveit_msgs::CollisionObject> collision_objects;
//TODO: consider adding top limit if necessary to stop over the top movements
		if(state == 1){//create collision object to constrain the rotate-to-wall movement to be as compact as possible.
			arc_limit = fabs(current_pose.position.y);
			if(fabs(current_pose.position.x) > fabs(current_pose.position.y)) arc_limit = fabs(current_pose.position.x);
			arc_limit = arc_limit+0.9; //0.5*width of EEF+small tolerance?
			if(arc_limit < 3.3) arc_limit = 3.3;
			printf("Arc_limit value: %f.\n", arc_limit);
			// Define a primative to add to the visualiser for the limit at the front of the robot. 
			std_msgs::ColorRGBA color;
			moveit_msgs::ObjectColor front; moveit_msgs::ObjectColor side;
			front.id = front_limit_object.id; side.id = side_limit_object.id; 
			front.color.r = 255; front.color.g = 255; front.color.b = 255; front.color.a = 0.3;
			side.color = front.color;
			std::vector<moveit_msgs::ObjectColor> color_vector = {front, side};		

			shape_msgs::SolidPrimitive front_limit;
			front_limit.type = front_limit.BOX;
			front_limit.dimensions.resize(3);
			front_limit.dimensions[0] = 6.0;
			front_limit.dimensions[1] = 0.01;
			front_limit.dimensions[2] = 6.0;
			
			geometry_msgs::Pose front_limit_pose;
			front_limit_pose.orientation.x = 0.0;
			front_limit_pose.orientation.y = 0.0;
			front_limit_pose.orientation.z = 0.0;
			front_limit_pose.orientation.w = 1.0;
			front_limit_pose.position.x = 0.0;
			front_limit_pose.position.y = arc_limit;
			front_limit_pose.position.z = front_limit.dimensions[2]*0.5;

		  	front_limit_object.primitives.push_back(front_limit);
		  	front_limit_object.primitive_poses.push_back(front_limit_pose);
		  	front_limit_object.operation = front_limit_object.ADD;
			collision_objects.push_back(front_limit_object);

			// Define a primative to add to the visualiser for the limit at the side of the robot. 
			shape_msgs::SolidPrimitive side_limit;
			side_limit.type = front_limit.BOX;
			side_limit.dimensions.resize(3);
			side_limit.dimensions[0] = 6.0;
			side_limit.dimensions[1] = 0.01;
			side_limit.dimensions[2] = 6.0;
			
			geometry_msgs::Pose side_limit_pose;
			side_limit_pose.orientation.x = 0.0;
			side_limit_pose.orientation.y = 0.0;
			if(surface_direction == "left") side_limit_pose.orientation.z = 0.7071068;
			else if(surface_direction == "right") side_limit_pose.orientation.z = -0.7071068;
			side_limit_pose.orientation.w = 0.7071068;
			if(surface_direction == "left") side_limit_pose.position.x = -1*arc_limit;
			else if(surface_direction == "right") side_limit_pose.position.x = arc_limit;
			side_limit_pose.position.y = 0.0;
			side_limit_pose.position.z = 2.5; //0.5*dimensions[2]

		  	side_limit_object.primitives.push_back(side_limit);
		  	side_limit_object.primitive_poses.push_back(side_limit_pose);
		  	side_limit_object.operation = side_limit_object.ADD;
			collision_objects.push_back(side_limit_object);

			planning_scene_interface.applyCollisionObjects(collision_objects, color_vector);
		}

		if(state == 2){//create painting surface object based on ArUco feedback.
			// Define a primative to add to the visualiser.
			shape_msgs::SolidPrimitive surface;
			surface.type = surface.BOX;
			surface.dimensions.resize(3);
			surface.dimensions[0] = 10.0;
			surface.dimensions[1] = 0.01;
			surface.dimensions[2] = surface_top_height;

		 	// Define a pose for the primative (specified relative to frame_id: anchor).
			geometry_msgs::Pose surface_pose;
			surface_pose.orientation = target_code.transform.rotation;
			surface_pose.position.x = wall_x;
			surface_pose.position.y = target_code.transform.translation.y;
			surface_pose.position.z = surface_top_height*0.5;


		  	wall_collision_object.primitives.push_back(surface);
		  	wall_collision_object.primitive_poses.push_back(surface_pose);
		  	wall_collision_object.operation = wall_collision_object.ADD;

			
	  		collision_objects.push_back(wall_collision_object);
			planning_scene_interface.applyCollisionObjects(collision_objects);
		}
	}


	bool hinowa_state::targetArucoAligned(){
		for(int i = 0; i < num_markers; i++){
			if(aruco_camera.at(i).header.seq == target_aruco_ID){
				target_code = aruco_camera.at(i);
				target_aruco_visible = true;
			}
		}
		if(target_aruco_visible){
			tf2::Vector3 V_aruco(target_code.transform.translation.x, target_code.transform.translation.y, target_code.transform.translation.z);
			tf2::Quaternion Q_aruco;
			tf2::convert(target_code.transform.rotation, Q_aruco);
			tf2::Vector3 arucoRPY;
			tf2::Matrix3x3(Q_aruco).getRPY(arucoRPY[0], arucoRPY[1], arucoRPY[2], 2);
			arucoRPY[0] = arucoRPY[0]*(180/3.14159); //convert to deg for meaningful tolerance definition.

			printf("Target code alignment errors: %f, %f.\n", V_aruco[0], arucoRPY[0]);

			if(((-T_aligned < V_aruco[0]) && (V_aruco[0] < T_aligned)) && ((-T_parallel < arucoRPY[0]) && (arucoRPY[0] < T_parallel))){
				target_aruco_aligned = true;
				update_wall = true;
			}
			else{
				printf("\033[0;31m");
				printf("/hinowa_control/hinowa_state.cpp/targetArucoAligned - Target ArUco code with ID: %d is within the FOV but not aligned with the EEF, reverting to Control State 0. Enable autonomous alignment by holding the right D-PAD button and square.\n",target_aruco_ID);
				printf("\033[0m"); 
				target_aruco_aligned = false;
			}
		}
		else{
			printf("\033[0;31m");
			printf("/hinowa_control/hinowa_state.cpp/targetArucoAligned - Target ArUco code with ID: %d is not within the FOV, reverting to Control State 0. Manually move the platform so this code is detected, then carry out autonomous alignment by holding the right D-PAD button and square.\n", target_aruco_ID);
			printf("\033[0m"); 
			target_aruco_aligned = false;
		}
		std::cout<<"Target_aruco_aligned: "<<target_aruco_aligned<<".\n";
		return target_aruco_aligned;
	}

	void hinowa_state::updateWallPoses(geometry_msgs::Pose current_pose){
		//camera frame: Z is optical distance, Y is height axis of image, X is width axis.
		//anchor frame: Z is vertical, y is parallel to wall, x is distance to wall.
		aruco_tuple = aruco_sub.subscribe();
		ros::spinOnce();
		this->num_markers = std::get<2>(aruco_tuple);
		//printf("Number of visible markers: %d.\n", num_markers);
		update_wall = false;
		target_aruco_visible = false;
		wall_x = 0.0;
		painting_x = 0.0;
		if(num_markers > 0){
			this->aruco_anchor = std::get<0>(aruco_tuple);
			this->aruco_camera = std::get<1>(aruco_tuple);
			if(targetArucoAligned()){
				for(int i = 0; i < num_markers; i++){
					wall_x += aruco_anchor.at(i).transform.translation.x;
				}
				wall_x = wall_x/num_markers;
				if(wall_x < 0) painting_x = wall_x + painting_distance;
				else painting_x = wall_x - painting_distance;
			}
		}
		else{
			printf("\033[0;31m");
			printf("/hinowa_control/hinowa_state.cpp/updateWallPoses - No ArUco codes are within the FOV, reverting to Control State 0. Manually move the platform so target ArUco code with ID: %d, then carry out autonomous alignment by holding the right D-PAD button and square.\n", target_aruco_ID);
			printf("\033[0m");
		}

//TODO: consider which option is best here - mirror the current pose orientation at the time of update, or look at transform between the code and the base, find the difference between that and perfect orientation of +-90deg around base, then offset the middle EEF joint to acheive exactly 90deg relative angle. Try implementing latter.

		approach_code.position.x = painting_x; 
		approach_code.position.y = current_pose.position.y; 
		approach_code.position.z = 1.0;//current_pose.position.z;
		approach_code.orientation.x = current_pose.orientation.x;
		approach_code.orientation.y = current_pose.orientation.y;
		approach_code.orientation.z = current_pose.orientation.z;
		approach_code.orientation.w = current_pose.orientation.w;


	//pose for top of surface
		surface_top.position.x = painting_x; //Aruco0_anchor x value is not absolute
		surface_top.position.y = 0.0; 
		surface_top.position.z = surface_top_height;
		surface_top.orientation.x = 0.0;
		surface_top.orientation.y = 0.0;
		if(surface_direction == "left") surface_top.orientation.z = 0.7071068;
		else if(surface_direction == "right") surface_top.orientation.z = -0.7071068;
		surface_top.orientation.w = 0.7071068;
		

	//pose for bottom of surface
	 	surface_bottom = surface_top;	
		surface_bottom.position.z = surface_bottom_height;
	}

	int hinowa_state::updateAction(int SELECT, int START, int PS){
		//printf("State: %d.\n", state);
		int currentAction = action;
		if((SELECT+START+PS) > 1) printf("[hinowa_control/hinowa_state.cpp::update_state] Press one button at a time when planning & executing, no input has been acknowledged.\n");
		else{//Only one button pressed. Determine actions based on which one, the current state & value of action variable. 
			int next_state = state+1; 
			if(state == (num_states-1)) next_state = 0; //handle wrapping state back to 0. 

			if(SELECT){//Prime action state.
				if(action == 1){
					action = 0; //If action state is currently primed, unprime it.
					std::cout<<"Current state: "<<stateInfo[state]<<". Next state: "<<stateInfo[next_state]<<", has been unprimed.\n";
				}
				else{ //action == -2 || action == -1 || action == 0 || action == 2 
					action = 1; //If any state other than already primed, prime advance action. 
					std::cout<<"Current state: "<<stateInfo[state]<<". Next state: "<<stateInfo[next_state]<<", has been primed.\n";
				}

			}
			else if(START){//Execute
				if(action == -1){
					action = -2;
					std::cout<<"Current state: "<<stateInfo[state]<<". Executing return to prior state: "; //Finish statement within action function.
				}
				else if(action == 1){
					action = 2;
					std::cout<<"Current state: "<<stateInfo[state]<<". Execution to next state: "<<stateInfo[next_state]<<" is being completed.\n";
				}
				//otherwise, do not execute. 
			}
			else if(PS){//Prime return to home.
				if(action == -1){
					action = 0; //If already primed for return to prime, unprime.
					std::cout<<"Current state: "<<stateInfo[state]<<". Planning return to prior state has been unprimed.\n";
				} 
				else{ //action == -2 || action == 0 || action == 1 || action == 2 
					action = -1; //If any state other than already primed return home, prime return home.
					std::cout<<"Current state: "<<stateInfo[state]<<". Planning return to prior state:";
				}
			}
		}
		//printf("Action: %d.\n", action);
		if(action == currentAction) update = 0;
		else update = 1;
		return update;

	}

	int hinowa_state::getState(){
		return this->state;	
	}

	void hinowa_state::setState(int new_state){
		this->state = new_state;	
	}

	int hinowa_state::getRegressState(){
		return this->regress_state;	
	}

	void hinowa_state::setRegressState(int regress_state){
		this->regress_state = regress_state;	
	}

	int hinowa_state::getTargetArucoID(){
		return this->target_aruco_ID;	
	}

	void hinowa_state::setTargetArucoID(int ID){
		this->target_aruco_ID = ID;	
	}

	void hinowa_state::startPaintingMovement(moveit::planning_interface::MoveGroupInterface& move_group){
		if(!(this->state == 4 && action == 2)) throw std::runtime_error("hinowa_state.cpp/startPainting can only be called when state = 4 (the lift is at the top of the surface and has planned the painting trajectory). Exiting for safety, check function call.\n");
		//ros::Duration(paint_delay).sleep();//paint_delay.sleep();
		ROS_INFO_NAMED("INFO","Execution command %s", (move_group.asyncExecute(this->plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ? "succeeded.\n" : "FAILED.\n");
		ros::Duration(paint_delay).sleep();//paint_delay.sleep();

	}

}



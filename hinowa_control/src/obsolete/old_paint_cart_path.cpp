//------//
/*
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
*/

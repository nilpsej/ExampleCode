cmake_minimum_required(VERSION 2.8.3)
project(hinowa_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_msgs
  moveit_planners_ompl
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_visualization
  moveit_visual_tools
  pluginlib
  geometric_shapes
  camera_tf2
  tf2
  tf2_ros
  hinowa_parameters
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hinowa_control_lib
  CATKIN_DEPENDS roscpp rospy std_msgs camera_tf2 tf2 tf2_ros
# DEPENDS system_lib

)

include_directories(
  include
  hinowa_control
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE)

add_library(hinowa_control_lib
  src/control_main.cpp
  src/hinowa_state.cpp
  src/control_subscriber.cpp
  src/execute_result_subscriber.cpp
  src/plan_result_subscriber.cpp
)

add_executable(control_loop src/control_main.cpp)

target_link_libraries(control_loop
  hinowa_control_lib
  ${catkin_LIBRARIES}
)

#add_dependencies(control_loop ${${PROJECT_NAME}_EXPORTED_TARGETS})


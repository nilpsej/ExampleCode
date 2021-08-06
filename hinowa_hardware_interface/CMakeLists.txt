cmake_minimum_required(VERSION 2.8.3)
project(hinowa_hardware_interface)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  hinowacpp
  joint_limits_interface
  joint_state_publisher
  joint_state_controller
  controller_manager
  hinowa_description
  robot_state_publisher
  joint_trajectory_controller
  camera_tf2
  hinowa_parameters
  hinowa_joint_errors
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hinowa_hardware_interface_lib
  CATKIN_DEPENDS roscpp rospy std_msgs camera_tf2 hinowacpp hinowa_joint_errors
#  DEPENDS system_lib
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
   src/hinowa_hardware_interface.cpp
)


add_executable(hinowa_hardware_interface_node src/hinowa_hardware_interface_node.cpp)



target_link_libraries(hinowa_hardware_interface_node
hinowa_hardware_interface_lib
${catkin_LIBRARIES}
)

#target_link_libraries(control_loop ${catkin_LIBRARIES})


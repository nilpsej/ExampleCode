cmake_minimum_required(VERSION 3.0.2)
project(hinowa_joint_errors)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files( FILES PID_terms.msg)
generate_messages( DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
   LIBRARIES hinowa_joint_errors_lib
   CATKIN_DEPENDS roscpp rospy message_runtime std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hinowa_joint_errors_lib
  src/joint_error_subscriber.cpp
  src/joint_error_publisher.cpp
)


add_dependencies(hinowa_joint_errors_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hinowa_joint_errors_lib ${catkin_LIBRARIES})

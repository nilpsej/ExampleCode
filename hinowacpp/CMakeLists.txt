cmake_minimum_required(VERSION 2.8.3)
project(hinowacpp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hinowa_can_msgs
  camera_tf2
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hinowacpp_lib
  CATKIN_DEPENDS
   hinowa_can_msgs
   camera_tf2
   roscpp
   rospy
   std_msgs
   tf2
   tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#PATTERN ".svn" EXCLUDE)

## Declare a C++ library
add_library(hinowacpp_lib
#	src/can_interface.cpp
	src/hinowa.cpp
#	src/main.cpp
	src/joint.cpp
	src/remote.cpp
	src/level_sensor.cpp
	src/status_subscriber.cpp
	src/can_rx_subscriber.cpp
	src/can_tx_publisher.cpp
	src/encoder_filter.cpp
)

add_dependencies(hinowacpp_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(hinowacpp_lib ${catkin_LIBRARIES})


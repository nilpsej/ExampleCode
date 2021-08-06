cmake_minimum_required(VERSION 2.8.3)
project(hinowa_record_home)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  hinowacpp
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hinowa_record_home_lib
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME}_lib
#   src/hinowa_hardware_record_home_node.cpp
#)

add_executable(hinowa_record_home_node src/hinowa_record_home_node.cpp)

target_link_libraries(hinowa_record_home_node
#hinowa_record_home_lib
${catkin_LIBRARIES}
)

target_link_libraries(hinowa_record_home_node ${catkin_LIBRARIES})

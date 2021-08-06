cmake_minimum_required(VERSION 2.8.3)
project(hinowa_parameters)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files( FILES hinowa_hwi.msg loop_control.msg)
generate_messages( DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hinowa_parameters_lib
   CATKIN_DEPENDS roscpp rospy message_runtime std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hinowa_parameters_lib
  src/ctrl_loop_publisher.cpp
  src/ctrl_loop_subscriber.cpp
  src/hwi_publisher.cpp
  src/hwi_subscriber.cpp
)

add_dependencies(hinowa_parameters_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hinowa_parameters_lib ${catkin_LIBRARIES})

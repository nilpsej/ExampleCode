cmake_minimum_required(VERSION 2.8.3)
project(hinowa_can)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hinowa_can_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hinowa_can_lib
  CATKIN_DEPENDS
    hinowa_can_msgs
    roscpp
    rospy
    std_msgs
# DEPENDS system_lib

)

include_directories(
  include
#  hinowa_control
  ${catkin_INCLUDE_DIRS}
)

add_library(hinowa_can_lib
  #src/actuate.cpp
  #src/can_sniffer.cpp
  src/can_rx.cpp
  #src/can_bcm.cpp
  src/can_raw.cpp
  #src/can_rx.cpp
)

#add_executable(can_rx_node src/can_rx_node.cpp)
add_executable(can_tx_node src/can_tx_node.cpp)
#add_executable(can_sniffer_node src/can_sniffer.cpp)
#add_executable(new_rx_node src/new_rx_node.cpp)
#add_executable(new_sniffer src/sniffer_unchanged.cpp)
add_executable(can_rx_node src/can_rx.cpp)

target_link_libraries(can_rx_node
  hinowa_can_lib 
  ${catkin_LIBRARIES}
)

#target_link_libraries(can_rx_node
#  hinowa_can_lib 
#  ${catkin_LIBRARIES}
#)

target_link_libraries(can_tx_node
  hinowa_can_lib 
  ${catkin_LIBRARIES}
)

#target_link_libraries(new_rx_node
#  hinowa_can_lib 
#  ${catkin_LIBRARIES}
#)

add_dependencies(hinowa_can_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hinowa_can_lib ${catkin_LIBRARIES})

cmake_minimum_required(VERSION 2.8.3)
project(bep_tpcw)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  wire_msgs
  problib
)

# Declare a catkin package                                                                                                                                
catkin_package(
  CATKIN_DEPENDS
    roscpp
    wire_msgs
    problib
)

include_directories(${catkin_INCLUDE_DIRS})

## Build

# ROS nodes
add_executable(get_world_state_TW src/get_world_state_TW.cpp) 
target_link_libraries(get_world_state_TW ${catkin_LIBRARIES})

add_executable(generate_evidence_TW src/generate_evidence_TW.cpp) 
target_link_libraries(generate_evidence_TW ${catkin_LIBRARIES})

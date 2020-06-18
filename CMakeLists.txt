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
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    wire_msgs
    problib
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Build

add_library(Room src/Room.cpp include/Room.h)
target_link_libraries(Room ${catkin_LIBRARIES})

add_library(Trajectories src/Trajectories.cpp include/Trajectories.h)
target_link_libraries(Trajectories ${catkin_LIBRARIES})

# ROS nodes
add_executable(generate_evidence_TW src/generate_evidence_TW.cpp) 
target_link_libraries(generate_evidence_TW Room Trajectories ${catkin_LIBRARIES})

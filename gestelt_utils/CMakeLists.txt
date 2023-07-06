cmake_minimum_required(VERSION 3.0.2)
project(gestelt_utils)

add_compile_options(-std=c++14)

set (ROS_LIBRARIES
  roscpp
  rospy
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS 
  include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${ROS_LIBRARIES}
#  DEPENDS Eigen Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/timebenchmark.cpp
)


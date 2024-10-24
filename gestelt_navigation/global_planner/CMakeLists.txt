cmake_minimum_required(VERSION 3.0.2)
project(global_planner)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


set(ROS_LIBRARIES
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  grid_map
  jps3d
  logger
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES global_planner # Exported libraries
 CATKIN_DEPENDS grid_map jps3d logger # Export grid_map as part of this package
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
  ${EIGEN3_INCLUDE_DIRS} 
)

## Declare a C++ library
add_library(global_planner 
  src/a_star.cpp
  src/jps_wrapper.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(global_planner
  ${catkin_LIBRARIES}
)

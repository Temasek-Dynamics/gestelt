cmake_minimum_required(VERSION 3.0.2)
project(global_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(ROS_LIBRARIES
  roscpp
  grid_map
  std_msgs
  geometry_msgs
  visualization_msgs
)

# set (ROS_DEPENDS

# )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES global_planner
#  CATKIN_DEPENDS ${ROS_DEPENDS}
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
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(a_star_lib 
  # src/planner_base.cpp
  src/a_star.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(a_star_lib
  ${catkin_LIBRARIES}
)

add_executable(a_star_planner_node src/a_star_node.cpp)
add_dependencies(a_star_planner_node
  ${a_star_lib_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(a_star_planner_node
  ${catkin_LIBRARIES}
  a_star_lib
)

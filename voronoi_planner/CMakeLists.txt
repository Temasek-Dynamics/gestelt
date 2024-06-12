cmake_minimum_required(VERSION 3.0.2)
project(voronoi_planner)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(ROS_LIBRARIES
  roscpp
  nav_msgs
  global_planner
  dynamic_voronoi
  gestelt_msgs
  grid_map
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)
find_package(Eigen3 REQUIRED)
find_package(SDL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES voronoi_planner # Exported libraries
#  CATKIN_DEPENDS 
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
add_executable(voronoi_planner_node
  src/voronoi_planner.cpp 
  src/voronoi_planner_node.cpp 
)
target_link_libraries(voronoi_planner_node 
  ${catkin_LIBRARIES}
  SDL SDL_image
)




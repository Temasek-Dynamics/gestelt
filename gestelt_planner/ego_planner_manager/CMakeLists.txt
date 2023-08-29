cmake_minimum_required(VERSION 3.0.2)
project(ego_planner_manager)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(ROS_PKGS
  roscpp
  std_msgs
  geometry_msgs
  trajectory_server_msgs
  grid_map
  traj_opt
  traj_utils
  swarm_benchmark
)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_PKGS}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner_manager
 CATKIN_DEPENDS ${ROS_PKGS}
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
)

add_library(ego_planner_manager
  src/ego_planner_manager.cpp
)
target_link_libraries(ego_planner_manager
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)



cmake_minimum_required(VERSION 3.0.2)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(ROS_LIBRARIES
  roscpp
  std_msgs
  geometry_msgs
  trajectory_server_msgs
  plan_env
  traj_opt
  traj_utils
  swarm_benchmark
)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${ROS_LIBRARIES}
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  # ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(ego_planner_node
  src/ego_planner_node.cpp 
)
target_link_libraries(ego_planner_node 
  ${PROJECT_NAME}
)



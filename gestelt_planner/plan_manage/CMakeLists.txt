cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


set(ROS_LIBRARIES
  roscpp
  std_msgs
  geometry_msgs
  plan_env
  traj_opt
  traj_utils
  trajectory_server_msgs
  gestelt_utils
)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
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

add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
)
target_link_libraries(ego_planner_node 
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
#add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})



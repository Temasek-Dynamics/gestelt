cmake_minimum_required(VERSION 3.0.2)
project(ego_planner_fsm)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(ROS_LIBRARIES
  roscpp
  std_msgs
  geometry_msgs
  tf2_ros
  trajectory_server_msgs
  # grid_map
  traj_opt
  # traj_utils
  # path_searching
  # swarm_benchmark
)

# set(ROS_DEPENDS
# )

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS ${ROS_DEPENDS}
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(ego_planner_fsm_node
  src/ego_planner_fsm_node.cpp 
  src/ego_planner_fsm.cpp 
  src/ego_planner_manager.cpp 
)
target_link_libraries(ego_planner_fsm_node 
  ${catkin_LIBRARIES}
)



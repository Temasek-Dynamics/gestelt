cmake_minimum_required(VERSION 3.0.2)
project(ego_planner_fsm)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++17 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


set(ROS_LIBRARIES
  roscpp
  std_msgs
  geometry_msgs
  tf2_ros
  gestelt_msgs
  traj_opt
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
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME} 
  include/ego_planner_fsm/ego_planner_manager.h
  src/ego_planner_manager.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(ego_planner_fsm_node
  src/ego_planner_fsm_node.cpp 
  src/ego_planner_fsm.cpp 
  src/ego_planner_manager.cpp 
)
target_link_libraries(ego_planner_fsm_node 
  ${catkin_LIBRARIES}
)


